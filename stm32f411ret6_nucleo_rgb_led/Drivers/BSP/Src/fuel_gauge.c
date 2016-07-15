/*
 ******************************************************************************
 * File Name          : FUEL_GAUGE.c
 * Description        : FUEL_GAUGE: RGB LED Driver - uses three pwm signals to control
 * 						voltages to the three leds making up an RGB led
 ******************************************************************************
 *
 * COPYRIGHT(c) 2016 SonicSensory Inc.
 *          All rights reserved. This code or any portion thereof
 *          may not be reproduced or used in any manner whatsoever
 *          without the express written permission of the SonicSensory Inc.
 *
 *Initialization Sequence
 * 1. Initialize Config Structure, RGB LED TIMER, PWM and GPIO Registers / Peripherals
 *    - FUEL_GAUGE_Init(rgbnum); //rgbnum is the RGB LED you are using (0 is functional rgb led and 1 is decorative rgb led)
 * 2. Start the RGB LED PWM generator - this generates three pwm outputs for the three individual leds of the rgb led
 *    - FUEL_GAUGE_Start(rgbnum); //rgbnum is the RGB LED number you are using
 *
 *Normal Operation
 * 1. Set the RGB color of the LED by setting the three pwm outputs (0 - 100%) of the RGB LED pins, this will update the voltages seen by the RGB LED
 *    FUEL_GAUGE_SetColor(rggnum, red, blue, green); //where red, blue, green is a number 0 - 255
 *
 *Shutdown Sequence
 * TODO: add notes
 *
 *Power-Down Sequence
 * TODO: add notes
 *
 * Pins Used:
 * PA9  -  PWM Output pin to Red LED of RGB LED (PA5 for the STM32F411RET6 nucleo board)
 * PC8  -  PWM Output pin to Green LED of RGB LED
 * PD13 - PWM Output pin to Blue LED of RGB LED (PC10 for the STM32F411RET6 nucleo board)
 *
 */

/*
 ********  INCLUDE FILES & EXTERNAL VARIABLES  ********
 */
#include "FUEL_GAUGE.h"


/*
 ********  DEFINE CONSTANTS & DATA TYPE (GLOBAL VARIABLES) ********
 */


/*
 ********  FUNCTION DECLARATION & DEFINTION ********
 */


/* ******************************************************************
** FUNCTION NAME: FUEL_GAUGE_InitConfigs()
 * DESCRIPTION: Initialize all RGB Config Structures
 * NOTE: Initialize the FUEL_GAUGE Structures
 *********************************************************************************** */
void FUEL_GAUGE_InitConfigs(int rgbnum)
{


}

/* ******************************************************************
** FUNCTION NAME: FUEL_GAUGE_Init()
 * DESCRIPTION: Initialize all RGB Led structures / timers
 * NOTE: Initialize the FUEL_GAUGE
 *********************************************************************************** */
void FUEL_GAUGE_Init(int rgbnum)
{

  return;
}

/* ********************************************************************************
** FUNCTION NAME: FUEL_GAUGE_DeInit()
** DESCRIPTION: FUEL_GAUGE to de-initialize the driver component.
** NOTE:		None.
*********************************************************************************** */
void FUEL_GAUGE_DeInit(int rgbnum)
{

}

/* ********************************************************************************
** FUNCTION NAME: FUEL_GAUGE_Reset()
** DESCRIPTION: Reset RGBx LED - stop pwm, configure data struct, initialize
** NOTE:		Does not call the FUEL_GAUGE_Start method - need to call after reset
*********************************************************************************** */
void FUEL_GAUGE_Reset(int rgbnum)
{

  return;
}



/* ********************************************************************************
** FUNCTION NAME: FUEL_GAUGE_Start()
** DESCRIPTION:
** NOTE:		None.
*********************************************************************************** */
void FUEL_GAUGE_Start(int rgbnum)
{
}


/* ********************************************************************************
** FUNCTION NAME: FUEL_GAUGE_Stop()
** DESCRIPTION: Stop the PWM generation on all color lines of RGBx LED
** NOTE:		None.
 *
*********************************************************************************** */
void FUEL_GAUGE_Stop(int rgbnum)
{

}



/* ********************************************************************************
** FUNCTION NAME: FUEL_GAUGE_PowerDown()
** DESCRIPTION: This power downs the driver
** NOTE:		None.
 *Power-Down Sequence:
*********************************************************************************** */
void FUEL_GAUGE_PowerDown(int rgbnum)
{

}


/* ********************************************************************************
** FUNCTION NAME: FUEL_GAUGE_ExitShutdown()
** DESCRIPTION:
** NOTE:		None.
 *
 *Exit:
*********************************************************************************** */
void FUEL_GAUGE_ExitShutdown(void)
{

}


/**
 * NAME: FUEL_GAUGE_EnterShutdown()
 *
 *Enter:
 */
void FUEL_GAUGE_EnterShutdown(void)
{

  return;
}



/* ********************************************************************************
** FUNCTION NAME: FUEL_GAUGE_ErrorHandler()
** DESCRIPTION: Handles Errors....
** NOTE:		None.
 *
*********************************************************************************** */
void FUEL_GAUGE_ErrorHandler(void)
{
	//TODO: Need to implement this method

  return;
}


