/*
 ******************************************************************************
 * File Name          : TLC_RGB_LED.c
 * Description        : TLC_RGB_LED: TLC59116 RGB LED Driver - 16 ch pwm controller for leds
 ******************************************************************************
 *
 * COPYRIGHT(c) 2016 SonicSensory Inc.
 *          All rights reserved. This code or any portion thereof
 *          may not be reproduced or used in any manner whatsoever
 *          without the express written permission of the SonicSensory Inc.
 *
 *Initialization Sequence
 * 1. Initialize Config Structure, RGB LED TIMER, PWM and GPIO Registers / Peripherals
 *    - TLC_RGB_LED_Init(rgbnum); //rgbnum is the RGB LED you are using (0 is functional rgb led and 1 is decorative rgb led)
 * 2. Start the RGB LED PWM generator - this generates three pwm outputs for the three individual leds of the rgb led
 *    - TLC_RGB_LED_Start(rgbnum); //rgbnum is the RGB LED number you are using
 *
 *Normal Operation
 * 1. TLC_RGB_LED_SetColor(int rgbnum, rgb_color_t * color)
 * 2. Sets the RGB color of the LED by setting the three pwm outputs (0 - 100%) of the RGB LED pins
 * 3. This will update the voltages seen by the RGB LED
 *
 *
 *
 *Shutdown / Power-Down Sequence
 * - TLC_RGB_LED_Stop
 * - TLC_RGB_LED_DeInit
 *
 *
 * Pins Used:
 * PA9  -  PWM Output pin to Red LED of RGB LED (PA5 for the STM32F411RET6 nucleo board)
 * PC8  -  PWM Output pin to Green LED of RGB LED
 * PD13 - PWM Output pin to Blue LED of RGB LED (PC10 for the STM32F411RET6 nucleo board)
 *
 * TO CHANGE PINS edit rgb_led.h:
 *   - RGB1_RED_PIN
 *   - RGB1_GREEN_PIN
 *   - RGB1_BLUE_PIN
 *
 */

/*
 ********  INCLUDE FILES & EXTERNAL VARIABLES  ********
 */
#include "tlc_rgb_led.h"


/*
 ********  DEFINE CONSTANTS & DATA TYPE (GLOBAL VARIABLES) ********
 */

//array of rgb led handlers
typedef struct tlc_rgb_led_status_t {
	int initialized;
	int error;
}tlc_rgb_led_status_t;

tlc_rgb_led_status_t tlcRgbLedStatus;
TLC_RGB_LED_handler_t RgbLedHandlers[TLC_RGB_LED_COUNT];


/*
 ********  FUNCTION DECLARATION & DEFINTION ********
 */


/* ******************************************************************
** FUNCTION NAME: TLC_RGB_LED_InitConfigs()
 * DESCRIPTION: Initialize all RGB Config Structures
 * NOTE: Initialize the RGB_LED Structures
 *********************************************************************************** */
void TLC_RGB_LED_InitConfigs(void)
{

	//Functional RGB LED Handler config
	RgbLedHandlers[FUNCTIONAL_RGB_NUM].red.channel = FUNCTIONAL_RGB_RED_CH;
	RgbLedHandlers[FUNCTIONAL_RGB_NUM].green.channel = FUNCTIONAL_RGB_GREEN_CH;
	RgbLedHandlers[FUNCTIONAL_RGB_NUM].blue.channel = FUNCTIONAL_RGB_BLUE_CH;

	//Decorative RGB LED1 Handler config
	RgbLedHandlers[DECORATIVE_RGB1_NUM].red.channel = DECORATIVE_RGB1_RED_CH;
	RgbLedHandlers[DECORATIVE_RGB1_NUM].green.channel = DECORATIVE_RGB1_GREEN_CH;
	RgbLedHandlers[DECORATIVE_RGB1_NUM].blue.channel = DECORATIVE_RGB1_BLUE_CH;

	//Decorative RGB LED2 Handler config
	RgbLedHandlers[DECORATIVE_RGB2_NUM].red.channel = DECORATIVE_RGB2_RED_CH;
	RgbLedHandlers[DECORATIVE_RGB2_NUM].green.channel = DECORATIVE_RGB2_GREEN_CH;
	RgbLedHandlers[DECORATIVE_RGB2_NUM].blue.channel = DECORATIVE_RGB2_BLUE_CH;

	//Decorative RGB LED1 Handler config
	RgbLedHandlers[DECORATIVE_RGB3_NUM].red.channel = DECORATIVE_RGB3_RED_CH;
	RgbLedHandlers[DECORATIVE_RGB3_NUM].green.channel = DECORATIVE_RGB3_GREEN_CH;
	RgbLedHandlers[DECORATIVE_RGB3_NUM].blue.channel = DECORATIVE_RGB3_BLUE_CH;

	//Decorative RGB LED1 Handler config
	RgbLedHandlers[DECORATIVE_RGB4_NUM].red.channel = DECORATIVE_RGB4_RED_CH;
	RgbLedHandlers[DECORATIVE_RGB4_NUM].green.channel = DECORATIVE_RGB4_GREEN_CH;
	RgbLedHandlers[DECORATIVE_RGB4_NUM].blue.channel = DECORATIVE_RGB4_BLUE_CH;

}

/*
 * Note: quick led on test
 */

void TLC_RGB_LED_Test(void){
	TLC_RGB_LED_Init(FUNC_TLC_RGB_LED_NUM);
	rgb_color_t color = {255, 255, 255};
	TLC_RGB_LED_SetColor(0, &color); //set all leds of rgb led 0 to full PWM ON
}
/*
 * Note: Check if was already initialized
 */

int TLC_RGB_LED_WasInitialized(void){
	return tlcRgbLedStatus.initialized;
}
/* ******************************************************************
** FUNCTION NAME: TLC_RGB_LED_Init()
 * DESCRIPTION: Initialize all RGB Led structures / timers
 * NOTE: Initialize the RGB_LED
 *********************************************************************************** */
void TLC_RGB_LED_Init(int rgbnum)
{
	TLC_RGB_LED_InitConfigs();
	TLC59116_init(); //init TI TLC59116 16 ch led controller

	//Once initialized, then ok to call TLC_RGB_LED_Start

  return;
}

/* ********************************************************************************
** FUNCTION NAME: TLC_RGB_LED_DeInit()
** DESCRIPTION: RGB_LED to de-initialize the driver component.
** NOTE:		None.
*********************************************************************************** */
void TLC_RGB_LED_DeInit(int rgbnum)
{
  /* Deinitialize RGB LED Driver */
  //TODO: TLC59116 de init not developed

}

/* ********************************************************************************
** FUNCTION NAME: TLC_RGB_LED_Reset()
** DESCRIPTION: Reset RGBx LED - stop pwm, configure data struct, initialize
** NOTE:		Does not call the TLC_RGB_LED_Start method - need to call after reset
*********************************************************************************** */
void TLC_RGB_LED_Reset(int rgbnum)
{
	TLC59116_reset(); //resets tlc 59116 led driver via i2c

  return;
}


/* ********************************************************************************
** FUNCTION NAME: TLC_RGB_LED_Start()
** DESCRIPTION:
** NOTE:		None.
*********************************************************************************** */
void TLC_RGB_LED_Start(int rgbnum)
{
	//TODO: currently led outputs already enabled in the tlc59116 init
	int redch = RgbLedHandlers[rgbnum].red.channel;
	int greench = RgbLedHandlers[rgbnum].green.channel;
	int bluech = RgbLedHandlers[rgbnum].blue.channel;
	TLC59116_start_led(redch);
	TLC59116_start_led(greench);
	TLC59116_start_led(bluech);
}

/* ********************************************************************************
** FUNCTION NAME: TLC_RGB_LED_StartLED()
** DESCRIPTION: Initialize individual LED TIM Handler and PWM Config structures
** NOTE:		None.
*********************************************************************************** */
void TLC_RGB_LED_StartLED(int rgbnum){
	//Activate outputs for specific rgb led
	//TODO: ?
}

/* ********************************************************************************
** FUNCTION NAME: TLC_RGB_LED_Stop()
** DESCRIPTION: Stop the PWM generation on all color lines of RGBx LED
** NOTE:		None.
 *
*********************************************************************************** */
void TLC_RGB_LED_Stop(int rgbnum)
{
	//TODO: ?

  return;
}

/* ********************************************************************************
** FUNCTION NAME: TLC_RGB_LED_StopLED()
** DESCRIPTION: Stop the PWM generation for a rgb led ( 3 leds)
** NOTE:		None.
 *
*********************************************************************************** */
void TLC_RGB_LED_StopLED(int rgbnum)
{
	//TODO: ?

  return;
}


/* ********************************************************************************
** FUNCTION NAME: TLC_RGB_LED_PowerDown()
** DESCRIPTION: This power downs the driver
** NOTE:		None.
 *Power-Down Sequence:
*********************************************************************************** */
void TLC_RGB_LED_PowerDown(int rgbnum)
{
	//TODO: ?

  return;
}


/* ********************************************************************************
** FUNCTION NAME: TLC_RGB_LED_ExitShutdown()
** DESCRIPTION:
** NOTE:		None.
 *
 *Exit:
*********************************************************************************** */
void TLC_RGB_LED_ExitShutdown(void)
{
	//TODO: ?
}


/**
 * NAME: TLC_RGB_LED_EnterShutdown()
 *
 *Enter:
 */
void TLC_RGB_LED_EnterShutdown(void)
{
	//TODO: ?
  return;
}

/* ********************************************************************************
** FUNCTION NAME: TLC_RGB_LED_SetColor()
** DESCRIPTION: set the three color pins pwm duty cycle of the rgb led to change color
** NOTE:		color variable is struct that just defines three int representing red, green and blue - range (0 - 255)
 *
*********************************************************************************** */
void TLC_RGB_LED_SetColor(int rgbnum, rgb_color_t * color)
{
	if(rgbnum >= TLC_RGB_LED_COUNT){
		TLC_RGB_LED_ErrorHandler();
		return;
	}
	//make sure rgb values between 0 - 255
	//red
	if(color->red < 0){color->red = 0;}
	else if(color->red > 255){color->red = 255;}
	//green
	if(color->green < 0){color->green = 0;}
	else if(color->green > 255){color->green = 255;}
	//blue
	if(color->blue < 0){color->blue = 0;}
	else if(color->blue > 255){color->blue = 255;}

	RgbLedHandlers[rgbnum].red.pwm = color->red;
	RgbLedHandlers[rgbnum].green.pwm = color->green;
	RgbLedHandlers[rgbnum].blue.pwm = color->blue;

	byte pwm_values[] = {RgbLedHandlers[rgbnum].red.pwm, RgbLedHandlers[rgbnum].green.pwm, RgbLedHandlers[rgbnum].blue.pwm};
	int start_ch = RgbLedHandlers[rgbnum].red.channel;
	int end_ch = RgbLedHandlers[rgbnum].blue.channel;
	TLC59116_set_outputs_from(pwm_values, start_ch, end_ch);

  return;
}

/* ********************************************************************************
** FUNCTION NAME: TLC_RGB_LED_GetColor()
** DESCRIPTION: get the current color settings of the rgb led
** NOTE:		color variable is struct that just defines three int representing red, green and blue - range (0 - 255)
 *
*********************************************************************************** */
void TLC_RGB_LED_GetColor(int rgbnum, rgb_color_t * color)
{
	if(rgbnum >= TLC_RGB_LED_COUNT){
		TLC_RGB_LED_ErrorHandler();
		return;
	}

	//RED - get red color rgb value 0 - 255
	color->red = RgbLedHandlers[rgbnum].red.pwm;
	//GREEN
	color->green = RgbLedHandlers[rgbnum].green.pwm;
	//BLUE
	color->blue = RgbLedHandlers[rgbnum].blue.pwm;

  return;
}

/* ********************************************************************************
** FUNCTION NAME: TLC_RGB_LED_Round()
** DESCRIPTION: Round number up / down (this is to save as much of the rgb led value after converting from float to int)
** NOTE:		example: TLC_RGB_LED_Round(1.5) = 2
 *
*********************************************************************************** */
int TLC_RGB_LED_Round(float value){
	float remainder = value - (int)value;
	//ex: if value = 1.9
	//    remainder = 1.9 - (int)1.9 = 1.9 - 1 = 0.9
	//    0.9 is > 0.5 so value = 1.9 + 0.5 = 2.3
	//   returns int(2.3) = 2
	if(remainder >= 0.5){
		value = value + 0.5;
		return value;
	}
	else{
		//ex: if value = 1.4 then just return int(1.4) = 1
		return value;
	}
}


/* ********************************************************************************
** FUNCTION NAME: TLC_RGB_LED_ErrorHandler()
** DESCRIPTION: Handles Errors....
** NOTE:		None.
 *
*********************************************************************************** */
void TLC_RGB_LED_ErrorHandler(void)
{
	//TODO: Need to implement this method

  return;
}






