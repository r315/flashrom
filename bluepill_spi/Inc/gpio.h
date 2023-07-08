#ifndef _gpio_h_
#define _gpio_h

#include <stdint.h>

#define GPIO_PIN_HIGH       1
#define GPIO_PIN_LOW        0

void GPIO_Config(uint32_t name, uint32_t cfg);
uint32_t GPIO_Read(uint32_t name);
void GPIO_Write(uint32_t name, uint32_t state);
void GPIO_Toggle(uint32_t name);
void GPIO_Function(uint32_t name, uint32_t func);
void GPIO_Mode(uint32_t name, uint32_t mode);
void GPIO_Direction(uint32_t name, uint32_t mode);
#endif