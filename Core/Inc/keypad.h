#ifndef KEYPAD_H
#define KEYPAD_H

#include "stm32l4xx_hal.h"

uint8_t keypad_init(I2C_HandleTypeDef *hi2c);
char keypad_get_key(void);
char keypad_get_key_raw(void);

#endif
