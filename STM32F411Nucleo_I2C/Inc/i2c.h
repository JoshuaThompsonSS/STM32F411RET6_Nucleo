/*
 * i2c.h
 * Desc: I2C Config and Functions
 */

#ifndef I2C_H_
#define I2C_H_
#include "stm32f4xx_hal.h"

#define I2C_BUFFERSIZE             32           // # of bytes for Tx & Rx buffers
I2C_HandleTypeDef hi2c1;

//Function declaration
void MX_I2C1_Init(void);

//Byte manipulation
unsigned int transBytes2UnsignedInt(unsigned char msb, unsigned char lsb);

#endif /* I2C_H_ */
