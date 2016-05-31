/* File Nameme          : rgb_led_interface.c
 * Description        : rgb_led_interface: RGB LED Driver Interface - wrapper around rgb_led driver to provide high level control of rgb leds
 ******************************************************************************
 *
 * COPYRIGHT(c) 2016 SonicSensory Inc.
 *          All rights reserved. This code or any portion thereof
 *          may not be reproduced or used in any manner whatsoever
 *          without the express written permission of the SonicSensory Inc.
 *
 *Initialization Sequence
 * TODO: add notes
 *
 *Normal Operation
 ** TODO: add notes
 *
 *Shutdown Sequence
 * TODO: add notes
 *
 *Power-Down Sequence
 * TODO: add notes
 *
 * Pins Used:
 * PA9  -  PWM Output pin to Red LED of RGB LED
 * PC8  -  PWM Output pin to Green LED of RGB LED
 * PD13 - PWM Output pin to Blue LED of RGB LED (on the STM32F411RET6 Nucleo Board use PC10)
 *
 */

/*
 ********  INCLUDE FILES & EXTERNAL VARIABLES  ********
 */
#include "functional_rgb_led.h"


/*
 ********  DEFINE CONSTANTS & DATA TYPE (GLOBAL VARIABLES) ********
 */



/*
 ********  FUNCTION DECLARATION & DEFINTION ********
 */
void FUNCTIONAL_RGB_LED_Init(void){
	return;
}
void FUNCTIONAL_RGB_LED_InitRampSeq(void){
	return;
}
void FUNCTIONAL_RGB_LED_InitHoldSeq(void){
	return;
}
void FUNCTIONAL_RGB_LED_InitSetpointSeq(void){
	return;
}
void FUNCTIONAL_RGB_LED_InitRepeatSeq(void){
	return;
}

void FUNCTIONAL_RGB_LED_Start(void)
{
	return;
}
void FUNCTIONAL_RGB_LED_Stop(void){
	return;
}
void FUNCTIONAL_RGB_LED_DeInit(void){
	return;
}
void FUNCTIONAL_RGB_LED_Reset(void){
	return;
}
void FUNCTIONAL_RGB_LED_SetColor(rgb_color_t * color){
	return;
}
void FUNCTIONAL_RGB_LED_GetColor(rgb_color_t * color){
	return;
}
void FUNCTIONAL_RGB_LED_GetColorDiff(rgb_color_t * color1, rgb_color_t * color2, rgb_color_t * colorDelta){
	return;
}

//Step functions to create specific RGB LED sequences
void FUNCTIONAL_RGB_LED_Ramp(void){
	return;
}
void FUNCTIONAL_RGB_LED_Hold(void){
	return;
}
void FUNCTIONAL_RGB_LED_Setpoint(void){
	return;
}
void FUNCTIONAL_RGB_LED_Repeat(void){
	return;
}

//Sequence routine service - should be called at least every 100 ms by SysTick_Handler in stm32f4xx_it.c
void FUNCTIONAL_RGB_LED_IRQHandler(void){
	return;
}


