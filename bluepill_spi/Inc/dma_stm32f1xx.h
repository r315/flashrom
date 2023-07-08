#ifndef _dma_stm32f1xx_h_
#define _dma_stm32f1xx_h_

#define DMA_NUMBER_POS     4
#define DMA_CHANNEL_POS    0

#define DMA_NUMBER_MASK    (3 << DMA_NUMBER_POS)
#define DMA_CHANNEL_MASK   (15 << DMA_CHANNEL_POS)


#define DMA1_MAX_CHANNELS   7
#define DMA2_MAX_CHANNELS   5
#define DMA_NUM_CHANNELS    (DMA1_MAX_CHANNELS + DMA2_MAX_CHANNELS)

/**
 * DMA Requests
 * */
#define DMA1_REQ_ADC1           ((0 << DMA_NUMBER_POS) | (0 << DMA_CHANNEL_POS))
#define DMA1_REQ_TIM2_CH3       ((0 << DMA_NUMBER_POS) | (0 << DMA_CHANNEL_POS))
#define DMA1_REQ_TIM4_CH1       ((0 << DMA_NUMBER_POS) | (0 << DMA_CHANNEL_POS))
#define DMA1_REQ_SPI1_RX        ((0 << DMA_NUMBER_POS) | (1 << DMA_CHANNEL_POS))
#define DMA1_REQ_USART3_TX      ((0 << DMA_NUMBER_POS) | (1 << DMA_CHANNEL_POS))
#define DMA1_REQ_TIM1_CH1       ((0 << DMA_NUMBER_POS) | (1 << DMA_CHANNEL_POS))
#define DMA1_REQ_TIM2_UP        ((0 << DMA_NUMBER_POS) | (1 << DMA_CHANNEL_POS))
#define DMA1_REQ_TIM3_CH3       ((0 << DMA_NUMBER_POS) | (1 << DMA_CHANNEL_POS))
#define DMA1_REQ_SPI1_TX        ((0 << DMA_NUMBER_POS) | (2 << DMA_CHANNEL_POS))
#define DMA1_REQ_USART3_RX      ((0 << DMA_NUMBER_POS) | (2 << DMA_CHANNEL_POS))
#define DMA1_REQ_TIM3_CH4       ((0 << DMA_NUMBER_POS) | (2 << DMA_CHANNEL_POS))
#define DMA1_REQ_TIM3_UP        DMA1_REQ_TIM3_CH4
#define DMA1_REQ_SPI2_RX        ((0 << DMA_NUMBER_POS) | (3 << DMA_CHANNEL_POS))
#define DMA1_REQ_I2S_RX         DMA1_REQ_SPI2_RX
#define DMA1_REQ_USART1_TX      ((0 << DMA_NUMBER_POS) | (3 << DMA_CHANNEL_POS))
#define DMA1_REQ_I2C2_TX        ((0 << DMA_NUMBER_POS) | (3 << DMA_CHANNEL_POS))
#define DMA1_REQ_TIM1_CH4       ((0 << DMA_NUMBER_POS) | (3 << DMA_CHANNEL_POS))
#define DMA1_REQ_TIM1_TRIG      DMA1_REQ_TIM1_CH4
#define DMA1_REQ_TIM1_COM       DMA1_REQ_TIM1_CH4
#define DMA1_REQ_TIM4_CH2       ((0 << DMA_NUMBER_POS) | (3 << DMA_CHANNEL_POS))
#define DMA1_REQ_SPI2_TX        ((0 << DMA_NUMBER_POS) | (4 << DMA_CHANNEL_POS))
#define DMA1_REQ_I2S_TX         DMA1_REQ_SPI2_TX
#define DMA1_REQ_USART1_RX      ((0 << DMA_NUMBER_POS) | (4 << DMA_CHANNEL_POS))
#define DMA1_REQ_I2C2_RX        ((0 << DMA_NUMBER_POS) | (4 << DMA_CHANNEL_POS))
#define DMA1_REQ_TIM1_UP        ((0 << DMA_NUMBER_POS) | (4 << DMA_CHANNEL_POS))
#define DMA1_REQ_TIM2_CH1       ((0 << DMA_NUMBER_POS) | (4 << DMA_CHANNEL_POS))
#define DMA1_REQ_TIM4_CH3       ((0 << DMA_NUMBER_POS) | (4 << DMA_CHANNEL_POS))
#define DMA1_REQ_USART2_RX      ((0 << DMA_NUMBER_POS) | (5 << DMA_CHANNEL_POS))
#define DMA1_REQ_I2C1_TX        ((0 << DMA_NUMBER_POS) | (5 << DMA_CHANNEL_POS))
#define DMA1_REQ_TIM1_CH3       ((0 << DMA_NUMBER_POS) | (5 << DMA_CHANNEL_POS))
#define DMA1_REQ_TIM3_CH1       ((0 << DMA_NUMBER_POS) | (5 << DMA_CHANNEL_POS))
#define DMA1_REQ_TIM3_TRIG      DMA1_REQ_TIM3_CH1
#define DMA1_REQ_USART2_TX      ((0 << DMA_NUMBER_POS) | (6 << DMA_CHANNEL_POS))
#define DMA1_REQ_I2C1_RX        ((0 << DMA_NUMBER_POS) | (6 << DMA_CHANNEL_POS))
#define DMA1_REQ_TIM2_CH2       ((0 << DMA_NUMBER_POS) | (6 << DMA_CHANNEL_POS))
#define DMA1_REQ_TIM2_CH4       DMA1_REQ_TIM2_CH2
#define DMA1_REQ_TIM4_UP        ((0 << DMA_NUMBER_POS) | (6 << DMA_CHANNEL_POS))

#define DMA2_REQ_SPI_RX         ((1 << DMA_NUMBER_POS) | (0 << DMA_CHANNEL_POS))
#define DMA2_REQ_I2S3_RX        DMA2_REQ_SPI_RX
#define DMA2_REQ_TIM5_CH4       ((1 << DMA_NUMBER_POS) | (0 << DMA_CHANNEL_POS))
#define DMA2_REQ_TIM5_TRIG      DMA2_REQ_TIM5_CH4
#define DMA2_REQ_TIM8_CH3       ((1 << DMA_NUMBER_POS) | (0 << DMA_CHANNEL_POS))
#define DMA2_REQ_TIM8_UP        DMA2_REQ_TIM8_CH3
#define DMA2_REQ_SPI_TX         ((1 << DMA_NUMBER_POS) | (1 << DMA_CHANNEL_POS))
#define DMA2_REQ_I2S3_TX        DMA2_REQ_SPI_TX
#define DMA2_REQ_TIM5_CH3       ((1 << DMA_NUMBER_POS) | (1 << DMA_CHANNEL_POS))
#define DMA2_REQ_TIM5_UP        DMA2_REQ_TIM5_CH3
#define DMA2_REQ_TIM8_CH4       ((1 << DMA_NUMBER_POS) | (1 << DMA_CHANNEL_POS))
#define DMA2_REQ_TIM8_TRIG      DMA2_REQ_TIM8_CH4
#define DMA2_REQ_TIM8_COM       DMA2_REQ_TIM8_CH4
#define DMA2_REQ_UART4_RX       ((1 << DMA_NUMBER_POS) | (2 << DMA_CHANNEL_POS))
#define DMA2_REQ_TIM6_UP        ((1 << DMA_NUMBER_POS) | (2 << DMA_CHANNEL_POS))
#define DMA2_REQ_DAC_CH1        DMA2_REQ_TIM6_UP
#define DMA2_REQ_TIM8_CH1       ((1 << DMA_NUMBER_POS) | (2 << DMA_CHANNEL_POS))
#define DMA2_REQ_SDIO           ((1 << DMA_NUMBER_POS) | (3 << DMA_CHANNEL_POS))
#define DMA2_REQ_TIM5_CH2       ((1 << DMA_NUMBER_POS) | (3 << DMA_CHANNEL_POS))
#define DMA2_REQ_TIM7_UP        ((1 << DMA_NUMBER_POS) | (3 << DMA_CHANNEL_POS))
#define DMA2_REQ_DAC_CH2        DMA2_REQ_TIM7_UP
#define DMA2_REQ_ADC3           ((1 << DMA_NUMBER_POS) | (4 << DMA_CHANNEL_POS))
#define DMA2_REQ_UART4_TX       ((1 << DMA_NUMBER_POS) | (4 << DMA_CHANNEL_POS))
#define DMA2_REQ_TIM5_CH1       ((1 << DMA_NUMBER_POS) | (4 << DMA_CHANNEL_POS))
#define DMA2_REQ_TIM8_CH2       ((1 << DMA_NUMBER_POS) | (4 << DMA_CHANNEL_POS))



#define DMA_CCR_PSIZE_8                 0
#define DMA_CCR_PSIZE_16                1
#define DMA_CCR_PSIZE_32                2
#define DMA_CCR_MSIZE_8                 0
#define DMA_CCR_MSIZE_16                1
#define DMA_CCR_MSIZE_32                2

#define DMA_CCR_PSIZE_CFG(x)            (x << 8)
#define DMA_CCR_MSIZE_CFG(x)            (x << 10)

#endif