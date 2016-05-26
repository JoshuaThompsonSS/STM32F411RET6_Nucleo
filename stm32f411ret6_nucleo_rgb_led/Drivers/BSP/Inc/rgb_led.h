/*
 * RGB_LED.h
 *
 */

#ifndef _RGB_LED_H_
#define _RGB_LED_H_
#include "stm32f4xx_hal.h"

void RGB_LED_Start(void);
void RGB_LED_Init(void);
void RGB_LED_DeInit(void);
void RGB_LED_Reset(void);
void RGB_LED_PowerDown(void);
void RGB_LED_SetConfig(void); //TODO: need config struct
void RGB_LED_ExitShutdown(void);
void RGB_LED_EnterShutdown(void);
void RGB_LED_SetColor(uint16_t red, uint16_t green, uint16_t blue);
void RGB_LED_SetOff(void);


#endif /* _RGB_LED_H_ */
