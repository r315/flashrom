#include "stm32f103xb.h"
#include "gpio_stm32f1xx.h"

static const uint32_t ports[] = {GPIOA_BASE, GPIOB_BASE, GPIOC_BASE, GPIOD_BASE, GPIOE_BASE};

/**
 * @brief Configure GPIO pin
 * @param port : pin port
 * @param pin : pin number 0 - 15
 * @param mode : Output
 *                  GPO_2MHZ
 *                  GPO_10MHZ
 *                  GPO_50MHZ
 *                  GPO_AF | GPO_xxMHZ
 *                  GPO_AF_OD | GPO_xxMHZ
 *               Input
 *                  GPI_FLOAT
 *                  GPI_ANALOG
 *                  GPI_PD
 *                  GPI_PU
 * */
void GPIO_Config(pinName_e name, uint8_t mode) {
    GPIO_TypeDef *port = (GPIO_TypeDef*)ports[GPIO_GET_PORT(name)];
    uint8_t pin = GPIO_GET_PIN(name);
    
    
    if(mode == GPI_PD){
        port->BRR = (1 << pin);
    }

    if(mode == GPI_PU){
        port->BSRR = (1 << pin);
    }    

    mode &= 0x0f;

    if(pin <  8){ 
        port->CRL = (port->CRL & ~(15 << (pin << 2))) | (mode << (pin << 2));
    }else{ 
        port->CRH = (port->CRH & ~(15 << ((pin - 8) << 2))) | (mode << ((pin - 8) << 2)); 
    }
}

uint32_t GPIO_Read(uint32_t name){
    GPIO_TypeDef *port = (GPIO_TypeDef*)ports[GPIO_GET_PORT(name)];
    uint8_t pin = GPIO_GET_PIN(name);

    return !!(port->IDR & (1 << pin));
}

void GPIO_Write(pinName_e name, uint8_t state){
    GPIO_TypeDef *port = (GPIO_TypeDef*)ports[GPIO_GET_PORT(name)];
    uint8_t pin = GPIO_GET_PIN(name);
    port->BSRR = (state != 0)? (1<<pin) : (1<<(pin+16));
}

void GPIO_Toggle(pinName_e name){
    GPIO_TypeDef *port = (GPIO_TypeDef*)ports[GPIO_GET_PORT(name)];
    uint8_t pin = GPIO_GET_PIN(name);
    port->ODR = port->IDR ^ (1<<pin);
}

void GPIO_PORT_Write(pinName_e name, uint32_t value){
    GPIO_TypeDef *port = (GPIO_TypeDef*)ports[GPIO_GET_PORT(name)];
    port->ODR = value;
}
uint32_t GPIO_PORT_Read(pinName_e name){
    GPIO_TypeDef *port = (GPIO_TypeDef*)ports[GPIO_GET_PORT(name)];
    return port->IDR;
}