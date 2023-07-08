#include <stdint.h>
#include "stm32f1xx.h"

#define WEAK            __attribute__((weak))
#define NORETURN        __attribute__((naked, __noreturn__, aligned(4)))
#define ISR             __attribute__((section(".isr_vector")))
#define DEFAULT_HANDLER __attribute__((weak, alias("Default_Handler")))

/* RCC_CR Bit Banding definitions 
    address = bit_banding_peripheral_base + (offset * 32) + (bit * 4)
*/
#define RCC_CR_HSEON_bb     (*(uint8_t *)0x42420040UL)
#define RCC_CR_HSERDY_bb    (*(uint8_t *)0x42420044UL)
#define RCC_CR_PLLON_bb     (*(uint8_t *)0x42420060UL)
#define RCC_CR_PLLRDY_bb    (*(uint8_t *)0x42420064UL)

ISR void *g_pfnVectors[];
uint32_t SystemCoreClock;

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _stack, _estack;


WEAK int main(void){}

static void errorHandler(void){
    while (1){
        asm("nop");
    }
}

NORETURN void Reset_Handler(void){
    volatile uint32_t *src, *dest;

    /* Copy initialize variables with .data section values*/
    for (src = &_sidata, dest = &_sdata; dest < &_edata; src++, dest++){
        *dest = *src;
    }

    /* Clear .bss */
    dest = &_sbss;

    while (dest < &_ebss)
        *dest++ = 0;

    /* Reset the RCC clock configuration to the default reset state(for debug purpose) */
    /* Set HSION bit */
    RCC->CR |= 0x00000001U;
    /* Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits */
    RCC->CFGR &= 0xF8FF0000U;
    /* Reset HSEON, CSSON and PLLON bits */
    RCC->CR &= 0xFEF6FFFFU;
    /* Reset HSEBYP bit */
    RCC->CR &= 0xFFFBFFFFU;
    /* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits */
    RCC->CFGR &= 0xFF80FFFFU;
    /* Disable all interrupts and clear pending bits  */
    RCC->CIR = 0x009F0000U;

#ifdef VECT_TAB_SRAM
    SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM. */
#else
    /* Vector Table Relocation to startup (g_pfnVectors) vector table  */
    //SCB->VTOR = FLASH_BASE;
    SCB->VTOR = (uint32_t)(&g_pfnVectors) & 0xFFFF;
#endif

    /* ------------- Configure system clock --------------- */
#define CLOCK_CFG_TIMEOUT 0x10000
    uint32_t timeout = CLOCK_CFG_TIMEOUT;

    /* Enable HSE */
    RCC->CR |= RCC_CR_HSEON;
    while ((RCC->CR & RCC_CR_HSERDY) == 0){
        timeout--;
        if (timeout == 0)
        {
            errorHandler();
        }
    }

    /* Configure and enable PLL oscillator (sysclk = 72Mhz) */
    
    RCC->CFGR = //(4 << 24) |                 // MCO = sysclk
#ifdef XTAL12MHZ
                (4 << 18) |                 // PLLMUL = 6
#else
                (7 << 18) |                 // PLLMUL = 9
#endif
                (1 << 16) |                 // PLLSRC = PREDIV1
                (2 << 14) |                 // ADCPRE = 6
                (4 << 8);                   // PPRE1 = 2

    timeout = CLOCK_CFG_TIMEOUT;
    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0){
        timeout--;
        if (timeout == 0)
        {
            errorHandler();
        }
    }

    /* Configure flash latency */
    FLASH->ACR |= 2;                        // Two wait states

    /* Switch  sysclk */
    RCC->CFGR |= (1 << 1);                  // select PLL as system clock
    timeout = CLOCK_CFG_TIMEOUT;
    while ((RCC->CFGR & (1 << 3)) == 0){
        timeout--;
        if (timeout == 0)
        {
            errorHandler();
        }
    }

    SystemCoreClock = 72000000UL;
    asm volatile("sub sp, sp, #64");         // Give some heap
 
    main();

    /* case returns... */
    asm("b .");
}

typedef struct stackframe_s{
    uint32_t r0, r1, r2, r3, r12, lr, pc, psr;
}stackframe_t;

void stackDump(stackframe_t *stack)
{
    asm volatile(
        "bkpt #01 \n"
        "b . \n"
    );
}

void HardFault_Handler(void){
    asm volatile
    (
        " tst lr, #4                                 \n"        // Check current stack
        " ite eq                                     \n"
        " mrseq r0, msp                              \n"        // Move msp to r0 ??
        " mrsne r0, psp                              \n"        // Move psp to r0 ??
        " ldr r1, [r0, #24]                          \n"        // Get address were exception happen ?
        " b stackDump                                \n"
    );
}

static void Default_Handler(void){
    volatile uint8_t isr_number = (SCB->ICSR & 0xFF) - 16;
    (void) isr_number;

    asm volatile(
        "bkpt #01 \n"
        "b ."
    );
}

void NMI_Handler(void)          DEFAULT_HANDLER;
//void HardFault_Handler(void)    DEFAULT_HANDLER;
void MemManage_Handler(void)    DEFAULT_HANDLER;
void BusFault_Handler(void)     DEFAULT_HANDLER;
void UsageFault_Handler(void)   DEFAULT_HANDLER;
void SVC_Handler(void)          DEFAULT_HANDLER;
void DebugMon_Handler(void)     DEFAULT_HANDLER;
void PendSV_Handler(void)       DEFAULT_HANDLER;
void SysTick_Handler(void)       DEFAULT_HANDLER;
void WWDG_IRQHandler(void)    DEFAULT_HANDLER;
void PVD_IRQHandler(void)    DEFAULT_HANDLER;
void TAMPER_IRQHandler(void)    DEFAULT_HANDLER;
void RTC_IRQHandler(void)    DEFAULT_HANDLER;
void FLASH_IRQHandler(void)    DEFAULT_HANDLER;
void RCC_IRQHandler(void)    DEFAULT_HANDLER;
void EXTI0_IRQHandler(void)    DEFAULT_HANDLER;
void EXTI1_IRQHandler(void)    DEFAULT_HANDLER;
void EXTI2_IRQHandler(void)    DEFAULT_HANDLER;
void EXTI3_IRQHandler(void)    DEFAULT_HANDLER;
void EXTI4_IRQHandler(void)    DEFAULT_HANDLER;
void DMA1_Channel1_IRQHandler(void)    DEFAULT_HANDLER;
void DMA1_Channel2_IRQHandler(void)    DEFAULT_HANDLER;
void DMA1_Channel3_IRQHandler(void)    DEFAULT_HANDLER;
void DMA1_Channel4_IRQHandler(void)    DEFAULT_HANDLER;
void DMA1_Channel5_IRQHandler(void)    DEFAULT_HANDLER;
void DMA1_Channel6_IRQHandler(void)    DEFAULT_HANDLER;
void DMA1_Channel7_IRQHandler(void)    DEFAULT_HANDLER;
void ADC1_2_IRQHandler(void)    DEFAULT_HANDLER;
void USB_HP_CAN1_TX_IRQHandler(void)    DEFAULT_HANDLER;
void USB_LP_CAN1_RX0_IRQHandler(void)    DEFAULT_HANDLER;
void CAN1_RX1_IRQHandler(void)    DEFAULT_HANDLER;
void CAN1_SCE_IRQHandler(void)    DEFAULT_HANDLER;
void EXTI9_5_IRQHandler(void)    DEFAULT_HANDLER;
void TIM1_BRK_IRQHandler(void)    DEFAULT_HANDLER;
void TIM1_UP_IRQHandler(void)    DEFAULT_HANDLER;
void TIM1_TRG_COM_IRQHandler(void)    DEFAULT_HANDLER;
void TIM1_CC_IRQHandler(void)    DEFAULT_HANDLER;
void TIM2_IRQHandler(void)    DEFAULT_HANDLER;
void TIM3_IRQHandler(void)    DEFAULT_HANDLER;
void TIM4_IRQHandler(void)    DEFAULT_HANDLER;
void I2C1_EV_IRQHandler(void)    DEFAULT_HANDLER;
void I2C1_ER_IRQHandler(void)    DEFAULT_HANDLER;
void I2C2_EV_IRQHandler(void)    DEFAULT_HANDLER;
void I2C2_ER_IRQHandler(void)    DEFAULT_HANDLER;
void SPI1_IRQHandler(void)    DEFAULT_HANDLER;
void SPI2_IRQHandler(void)    DEFAULT_HANDLER;
void USART1_IRQHandler(void)    DEFAULT_HANDLER;
void USART2_IRQHandler(void)    DEFAULT_HANDLER;
void USART3_IRQHandler(void)    DEFAULT_HANDLER;
void EXTI15_10_IRQHandler(void)    DEFAULT_HANDLER;
void RTC_Alarm_IRQHandler(void)    DEFAULT_HANDLER;
void USBWakeUp_IRQHandler(void)    DEFAULT_HANDLER;

ISR void *g_pfnVectors[] = {
    &_estack,
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0,
    0,
    0,
    0,
    SVC_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,
    WWDG_IRQHandler,
    PVD_IRQHandler,
    TAMPER_IRQHandler,
    RTC_IRQHandler,
    FLASH_IRQHandler,
    RCC_IRQHandler,
    EXTI0_IRQHandler,
    EXTI1_IRQHandler,
    EXTI2_IRQHandler,
    EXTI3_IRQHandler,
    EXTI4_IRQHandler,
    DMA1_Channel1_IRQHandler,
    DMA1_Channel2_IRQHandler,
    DMA1_Channel3_IRQHandler,
    DMA1_Channel4_IRQHandler,
    DMA1_Channel5_IRQHandler,
    DMA1_Channel6_IRQHandler,
    DMA1_Channel7_IRQHandler,
    ADC1_2_IRQHandler,
    USB_HP_CAN1_TX_IRQHandler,
    USB_LP_CAN1_RX0_IRQHandler,
    CAN1_RX1_IRQHandler,
    CAN1_SCE_IRQHandler,
    EXTI9_5_IRQHandler,
    TIM1_BRK_IRQHandler,
    TIM1_UP_IRQHandler,
    TIM1_TRG_COM_IRQHandler,
    TIM1_CC_IRQHandler,
    TIM2_IRQHandler,
    TIM3_IRQHandler,
    TIM4_IRQHandler,
    I2C1_EV_IRQHandler,
    I2C1_ER_IRQHandler,
    I2C2_EV_IRQHandler,
    I2C2_ER_IRQHandler,
    SPI1_IRQHandler,
    SPI2_IRQHandler,
    USART1_IRQHandler,
    USART2_IRQHandler,
    USART3_IRQHandler,
    EXTI15_10_IRQHandler,
    RTC_Alarm_IRQHandler,
    USBWakeUp_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
};