/*
 * i2c.h
 * Desc: I2C Config and Functions
 */

#ifndef I2C_H_
#define I2C_H_
#include "stm32f4xx_hal.h"


I2C_HandleTypeDef hi2c1;

//Function declaration
void MX_I2C1_Init(void);
#endif /* I2C_H_ */
