#include <stdint.h>
#include "bluepill_spi.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "fifo.h"
#include "spi.h"
#include "gpio.h"
#include "stimer.h"

typedef enum flashrom_e{
    IDLE = 0,
    HDR,
    RCV,
    SDN
}flashrom_t;

static uint8_t rx_data[512];
static fifo_t rx_fifo;
static spidrv_t spidrv;
static flashrom_t state = IDLE;
static simpletimer_t htimer;
static uint32_t SystemTic;

static void reenumerate_usb(void);
static void SPI_Per_Init(void);

static void SPI_Exchange(uint8_t *buf, uint32_t len)
{
    GPIO_Write(PB_12, 0);
    while(len--){
        *buf = SPI_Xchg(&spidrv, buf);
        buf++;
    }
    GPIO_Write(PB_12, 1);
}

static uint32_t resetState(simpletimer_t *timer)
{
    state = IDLE;
    return 0;
}

int main(void)
{
    uint8_t data;
    uint16_t idx;
    uint16_t xfer_count;
    uint8_t buf[16];

    reenumerate_usb();

    SysTick_Config(SystemCoreClock / 1000);

    fifo_init(&rx_fifo, rx_data, sizeof(rx_data));

    USB_Device_Init();

    SPI_Per_Init();

    STIMER_Config(&htimer, 1000, resetState);

    idx = xfer_count = 0;

    while(1){
        if(fifo_get(&rx_fifo, &data) == 0){
            continue;
        }        

        switch (state){
            case IDLE:            
                switch(data){                    
                    case ':':                        
                        state = HDR;
                        STIMER_Start(&htimer);                        
                        break;
                    default:
                        break;
                }
                break;            

            case HDR:
                xfer_count = data;
                idx = 0;
                state = RCV;
                break;

            case RCV:
                buf[idx++] = data;

                if(idx < xfer_count){
                    break;
                }

                STIMER_Start(&htimer);
                SPI_Exchange(buf, xfer_count);
                CDC_Transmit_FS(buf, xfer_count);
                state = IDLE;
                break;

            case SDN:                
                state = IDLE;
                break;

            default:
                break;

        }
    }

    return 0;
}

/**
 * 
*/
void CDC_Receive(uint8_t *Buf, uint16_t Len)
{
    if(fifo_free(&rx_fifo) < Len){
        return;
    }

    while(Len--){
        fifo_put(&rx_fifo, *Buf++);
    }
}

void SysTick_Handler(void)
{
    SystemTic++;
    STIMER_Handler(SystemTic);
}

static void reenumerate_usb(void){
    USB->CNTR = USB_CNTR_PDWN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    GPIOA->CRH = (GPIOA->CRH & ~(0x0F << 16)) | (2 << 16);
    GPIOA->BRR |= (1 << 12);
}

static void SPI_Per_Init(void)
{
    spidrv.bus = SPI_BUS1;
    spidrv.freq = 1000;
    spidrv.flags = SPI_IDLE;

    SPI_Init(&spidrv);

    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    GPIO_Config(PB_15, GPO_AF | GPO_50MHZ);
    GPIO_Config(PB_14, GPO_AF | GPO_50MHZ);
    GPIO_Config(PB_13, GPO_AF | GPO_50MHZ);
    GPIO_Config(PB_12, GPO_2MHZ);
    
    GPIO_Write(PB_12, 1);
}