/*
 ******************************************************************************
 * File Name          : RGB_LED.c
 * Description        : RGB_LED: RGB LED Driver - uses three pwm signals to control
 * 						voltages to the three leds making up an RGB led
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
#if 1

/*
 ********  INCLUDE FILES & EXTERNAL VARIABLES  ********
 */
#include "rgb_led.h"

/*
 ********  DEFINE CONSTANTS & DATA TYPE (GLOBAL VARIABLES) ********
 */


/*
 ********  FUNCTION DECLARATION & DEFINTION ********
 */

/* ********************************************************************************
** FUNCTION NAME: RGB_LED_Start()
** DESCRIPTION:
** NOTE:		None.
*********************************************************************************** */
void RGB_LED_Start()
{

}

/* ******************************************************************
** FUNCTION NAME: RGB_LED_Init()
 *
 * comment: Initialize the RGB_LED
 *********************************************************************************** */
void RGB_LED_Init()
{

  return;
}

/* ********************************************************************************
** FUNCTION NAME: RGB_LED_DeInit()
** DESCRIPTION: RGB_LED to de-initialize the driver component.
** NOTE:		None.
*********************************************************************************** */
void RGB_LED_DeInit(void)
{
  /* Deinitialize RGB LED interface */
}

/* ********************************************************************************
** FUNCTION NAME: RGB_LED_Reset()
** DESCRIPTION: Reset RGB_LED. To possibly turn into MACRO later if we deem other action are not required.
** NOTE:		None.
*********************************************************************************** */
void RGB_LED_Reset(void)
{
  /* Initialize the Control interface of the RGB LED Driver */

  /* TODO: */

  return;
}

/* ********************************************************************************
** FUNCTION NAME: RGB_LED_PowerDown()
** DESCRIPTION: This power downs the driver
** NOTE:		None.
 *Power-Down Sequence:
*********************************************************************************** */
void RGB_LED_PowerDown(void)
{

  return;
}

/* ********************************************************************************
** FUNCTION NAME: RGB_LED_Configuration()
** DESCRIPTION: Set of configuration of LED output model
** NOTE:		None.
*********************************************************************************** */
void RGB_LED_SetConfig(void)
{
  /* TODO: need to define config struct and pass that as argument */

}

/* ********************************************************************************
** FUNCTION NAME: RGB_LED_ExitShutdown()
** DESCRIPTION:
** NOTE:		None.
 *
 *Exit:
*********************************************************************************** */
void RGB_LED_ExitShutdown(void)
{

}


/**
 * NAME: RGB_LED_EnterShutdown()
 *
 *Enter:
 */
void RGB_LED_EnterShutdown(void)
{

  return;
}

/* ********************************************************************************
** FUNCTION NAME: RGB_LED_SetColor()
** DESCRIPTION:
** NOTE:		None.
 *
 *Exit:
*********************************************************************************** */
void RGB_LED_SetColor(uint16_t red, uint16_t green, uint16_t blue)
{

  return;
}

/* ********************************************************************************
** FUNCTION NAME: RGB_LED_SetOff()
** DESCRIPTION:
** NOTE:		None.
 *
 *Exit:
*********************************************************************************** */
void RGB_LED_SetOff(void)
{

  return;
}

#endif
