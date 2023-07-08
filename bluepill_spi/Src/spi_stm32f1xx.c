#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "stm32f103xb.h"
#include "spi.h"
#include "dma.h"
#include "dma_stm32f1xx.h"


#define PCLK_CLK_DIV2           (0)
#define PCLK_CLK_DIV4           (1 << 3)
#define PCLK_CLK_DIV8           (2 << 3)
#define PCLK_CLK_DIV16          (3 << 3)
#define PCLK_CLK_DIV32          (4 << 3)
#define PCLK_CLK_DIV64          (5 << 3)
#define PCLK_CLK_DIV128         (6 << 3)
#define PCLK_CLK_DIV256         (7 << 3)
#define SPIDEV_SET_FLAG(_D, _F) _D->flags |= _F
#define SPIDEV_CLR_FLAG(_D, _F) _D->flags &= ~(_F)
#define SPIDEV_GET_FLAG(_D, _F) !!(_D->flags & _F)

static spidrv_t *spidrv_l[2];

/**
 * @brief DMA Interrupt handler
 * */
void SPI_DMA_IRQHandler(spidrv_t *spidev){
    SPI_TypeDef *spi = (SPI_TypeDef*)spidev->ctrl;
    DMA_Channel_TypeDef *dma = (DMA_Channel_TypeDef*)spidev->dma.stream;
    
    dma->CCR &= ~(DMA_CCR_EN);
        
    if(spidev->trf_counter > 0x10000UL){
        spidev->trf_counter -= 0x10000UL;
        dma->CNDTR = (spidev->trf_counter > 0x10000UL) ? 0xFFFFUL : spidev->trf_counter;
        dma->CCR |= DMA_CCR_EN;
    }else{
        if(spi->SR & SPI_SR_OVR){
            //dummy read for clearing OVR flag
            spidev->trf_counter = spi->DR;
        }
    
        spi->CR2 &= ~(SPI_CR2_TXDMAEN);

        spidev->trf_counter = 0;

        if(spidev->eot_cb){
            spidev->eot_cb();
        }

        SPIDEV_CLR_FLAG(spidev, SPI_BUSY | SPI_DMA_NO_MINC);
    }
}

static inline void spi1Eot(void){ SPI_DMA_IRQHandler(spidrv_l[0]);}
static inline void spi2Eot(void){ SPI_DMA_IRQHandler(spidrv_l[1]);}

static uint32_t getPclk(uint8_t bus)
{
    uint8_t div = ((bus == 1) ? RCC->CFGR >> 8 : RCC->CFGR >> 11) & 7;
    
    return (div < 4) ? SystemCoreClock : (SystemCoreClock >> (1 << (div & 3)));
}
/**
 * @brief Configures baud rate by dividing Fpckl
 * by 2, 4, 8, 16, 32, 64, 128 or 256.
 * 
 * spi peripheral must be enabled afterwards 
 * 
 * */
static void SPI_SetFreq(SPI_TypeDef *spi, uint32_t pclk, uint32_t freq){

    uint32_t div = (pclk/1000)/freq;
    uint32_t br = 8;

    if(div > 256){
        div = 256;
    }

    if(div < 2){
        div = 2;
    }

    do{
        if(div >= (2 << br)){
            break;
        }
        br--;        
    }while(br);

    spi->CR1 &= ~(SPI_CR1_SPE | PCLK_CLK_DIV256);
    spi->CR1 |= (br << SPI_CR1_BR_Pos);
}
/**
 * Public API
 * */

/**
 * @brief Init
 * */
void SPI_Init(spidrv_t *spidrv){
    SPI_TypeDef *spi;
    uint8_t dma_req;
    uint32_t pclk;
    
    switch(spidrv->bus){
        case SPI_BUS0:
            __HAL_RCC_SPI1_CLK_ENABLE();
            __HAL_RCC_SPI1_FORCE_RESET();
            __HAL_RCC_SPI1_RELEASE_RESET();
            pclk = getPclk(2);
            spi = SPI1;
            dma_req = DMA1_REQ_SPI1_TX;
            spidrv_l[0] = spidrv;
            break;

        case SPI_BUS1:
            __HAL_RCC_SPI2_CLK_ENABLE();
            __HAL_RCC_SPI2_FORCE_RESET();
            __HAL_RCC_SPI2_RELEASE_RESET();
            pclk = getPclk(1);
            spi = SPI2;
            dma_req = DMA1_REQ_SPI2_TX;
            spidrv_l[1] = spidrv;
            break;

        default : return;
    }
    
    __HAL_RCC_DMA1_CLK_ENABLE();

    spi->CR1 = SPI_CR1_MSTR;
    
    SPI_SetFreq(spi, pclk, spidrv->freq);

    if((spidrv->flags & SPI_HW_CS) == 0){
        // In master mode SSOE must be enable for NSS
        // software control
        spi->CR2 |=  SPI_CR2_SSOE;
    }            

    spi->CR1 |= SPI_CR1_SPE;
   
    spidrv->dma.dst = (void*)&spi->DR;
    spidrv->dma.dsize = DMA_CCR_PSIZE_16;
    spidrv->dma.src = (void*)0;
    spidrv->dma.ssize = DMA_CCR_MSIZE_16;
    spidrv->dma.dir = DMA_DIR_M2P;
    spidrv->dma.eot = (spi == SPI1) ? spi1Eot : spi2Eot;
    
    DMA_Config(&spidrv->dma, dma_req);

    spidrv->ctrl = spi;
    spidrv->flags |= SPI_ENABLED;
}

/**
 * @brief Make single data exchange on spi bus
 *
 * \param spidev : Pointer to spi device to be used
 * \param data  : Data to be transmitted
 *
 * \return Received data
 * */
uint16_t SPI_Xchg(spidrv_t *spidev, uint8_t *data){
    SPI_TypeDef *spi = (SPI_TypeDef*)spidev->ctrl;

    if(spidev->flags & SPI_16BIT){
        spi->CR1 |= SPI_CR1_DFF;
        *((__IO uint16_t *)&spi->DR) = *(uint16_t*)data;
    }else{
        spi->CR1 &= ~SPI_CR1_DFF;
        *((__IO uint8_t *)&spi->DR) = *data;
    }

    while((spi->SR & SPI_SR_TXE) == 0);
    while((spi->SR & SPI_SR_BSY) != 0);

    return spi->DR;
}

/**
 * @brief Write data to SPI, blocking
 * 
 * \param src   : Pointer to source data
 * \param count : total number of bytes to transfer
 * */
void SPI_Transfer(spidrv_t *spidev, uint8_t *src, uint32_t count){
    SPI_TypeDef *spi = (SPI_TypeDef*)spidev->ctrl;
    
    if(spidev->flags & SPI_16BIT){
        spi->CR1 |= SPI_CR1_DFF;
        while(count--){
            *((__IO uint16_t *)&spi->DR) = *(uint16_t*)src++;
            while((spi->SR & SPI_SR_TXE) == 0);
            while((spi->SR & SPI_SR_BSY) != 0);
        }
    }else{
        spi->CR1 &= ~SPI_CR1_DFF;
        while(count--){
            *((__IO uint8_t *)&spi->DR) = *src++;
            while((spi->SR & SPI_SR_TXE) == 0);
            while((spi->SR & SPI_SR_BSY) != 0);
        }
    }
}

/**
 * @brief Write data to SPI using DMA controller
 * 
 * \param data  : Pointer to data
 * \param count : total number of transfers
 * */
void SPI_TransferDMA(spidrv_t *spidev, uint8_t *src, uint32_t count){
    static uint16_t sdata;
    SPI_TypeDef *spi = (SPI_TypeDef*)spidev->ctrl;
    DMA_Channel_TypeDef *dma = (DMA_Channel_TypeDef*)spidev->dma.stream;

    if(spidev->flags & SPI_16BIT){
        spi->CR1 |= SPI_CR1_DFF;
    }else{
        spi->CR1 &= ~SPI_CR1_DFF;
    }

    if(spidev->flags & SPI_DMA_NO_MINC){
        sdata = *(uint16_t*)src;
        src = (uint8_t*)&sdata;
        dma->CCR &= ~(DMA_CCR_MINC);
    }else{
        dma->CCR |= DMA_CCR_MINC;
    }

    spi->CR2 |= SPI_CR2_TXDMAEN;

    spidev->trf_counter = count;

    dma->CMAR = (uint32_t)src;
    dma->CNDTR = (spidev->trf_counter > 0x10000) ? 0xFFFF : spidev->trf_counter;
    
    dma->CCR |= DMA_CCR_EN;

    SPIDEV_SET_FLAG(spidev, SPI_BUSY);
}

/**
 * @brief Wait for end of DMA transfer
 * */
void SPI_WaitEOT(spidrv_t *spidev){    
    #if 1
    SPI_TypeDef *spi = (SPI_TypeDef*)spidev->ctrl;
    while(spi->SR & SPI_SR_BSY){
    #else
    while(SPIDEV_GET_FLAG(spidev, SPI_BUSY)){
    #endif
        //LED_TOGGLE;
    }
}

