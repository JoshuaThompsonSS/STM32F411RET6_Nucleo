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

/* ******************************************************************
** FUNCTION NAME: RGB_LED_Init()
 *
 * comment: Initialize the RGB_LED
 *********************************************************************************** */
void RGB_LED_Init()
{
	//TODO: add ability to init different RGB LEDs (right now just initializing RGB LED1 manually)
	//Maybe add argument to this function to init specified RGB LED..?

	//Init RGB1 LED Handler and Config
	RGB_LED_InitLEDConfig(&RGB1_RedTimHandle, &RGB1_RedPwmConfig, RGB1_RED_TIM_REG, RGB1_RED_TIM_CH);
	RGB_LED_InitLEDConfig(&RGB1_GreenTimHandle, &RGB1_GreenPwmConfig, RGB1_GREEN_TIM_REG, RGB1_GREEN_TIM_CH);
	RGB_LED_InitLEDConfig(&RGB1_BlueTimHandle, &RGB1_BluePwmConfig, RGB1_BLUE_TIM_REG, RGB1_BLUE_TIM_CH);

	//Once initialized, then ok to call RGB_LED_Start

  return;
}


/* ********************************************************************************
** FUNCTION NAME: RGB_LED_InitLEDConfig()
** DESCRIPTION: Initialize individual LED TIM Handler and PWM Config structures
** NOTE:		None.
*********************************************************************************** */
void RGB_LED_InitLEDConfig(TIM_HandleTypeDef * TimHandlePtr, TIM_OC_InitTypeDef * PwmConfigPtr, TIM_TypeDef * TimReg, uint32_t TimChannel){
	TimHandlePtr->Instance = TimReg;
	//timer_freq = system clock / (prescaler + 1)
	//Period(Cycles) = Pwm Period * timer_freq = timer_freq / pwm freq  --- where Pwm period = 1 / pwm freq
	uint32_t timer_freq = HAL_RCC_GetSysClockFreq() / RGB_PRESCALER_DFLT; // not including the + 1 ...just because! I may have reasons
	uint32_t timer_period = timer_freq / RGB_PWMFREQ_DFLT;
	TimHandlePtr->Init.Period = timer_period;
	TimHandlePtr->Init.Prescaler = RGB_PRESCALER_DFLT;
	//pwm freq = timer period / timer_freq = 1000 Hz (or whatever is defined by RGB_PWMFREQ_DFLT)
	TimHandlePtr->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	TimHandlePtr->Init.CounterMode = TIM_COUNTERMODE_UP;
	TimHandlePtr->Init.RepetitionCounter = 0;

	//configure TIM mode for pwm function generator
	//HAL_TIM_PWM_MspInit(&TimHandle);
	if(HAL_TIM_PWM_Init(TimHandlePtr) != HAL_OK)
	{
		/* Configuration Error */
		RGB_LED_ErrorHandler();
	}

	PwmConfigPtr->OCMode = TIM_OCMODE_PWM1;
	PwmConfigPtr->OCFastMode = TIM_OCFAST_ENABLE;
	PwmConfigPtr->OCPolarity = TIM_OCPOLARITY_HIGH;
	PwmConfigPtr->Pulse = RGB_PWMDUTY_DFLT * timer_period / 100;  //Pulse(Cycles) = Duty_Cycle * Period(Cycles) / 100 ex: 50% * 16000 / 100 = 8000
	if(HAL_TIM_PWM_ConfigChannel(TimHandlePtr, PwmConfigPtr, TimChannel) != HAL_OK)
	{
		/* Configuration Error */
		RGB_LED_ErrorHandler();
	}
}

/* ********************************************************************************
** FUNCTION NAME: RRGB_LED_InitLEDPwm()
** DESCRIPTION:
** NOTE:		This method is called by the HAL_TIM_PWM_MspInit callback
*********************************************************************************** */
void RGB_LED_InitLEDPwm(GPIO_InitTypeDef * GPIO_InitStructPtr, GPIO_TypeDef * LED_PORT, uint16_t LED_PIN){
	//Enable GPIOx peripheral clock
	if(LED_PORT == GPIOA){
		__HAL_RCC_GPIOA_CLK_ENABLE();
	}
	else if(LED_PORT == GPIOB){
		__HAL_RCC_GPIOB_CLK_ENABLE();
	}
	else if(LED_PORT == GPIOC){
		__HAL_RCC_GPIOC_CLK_ENABLE();
	}
	else if(LED_PORT == GPIOD){
			__HAL_RCC_GPIOD_CLK_ENABLE();
		}
	else if(LED_PORT == GPIOE){
			__HAL_RCC_GPIOE_CLK_ENABLE();
		}
	else{
		//GPIOH is not available for Timer based PWM generation
		RGB_LED_ErrorHandler();
	}


	//Enable GPIOx pin as output and assign pin number
	GPIO_InitStructPtr->Pin = LED_PIN;
	GPIO_InitStructPtr->Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructPtr->Pull = GPIO_NOPULL;//GPIO_PULLDOWN;
	GPIO_InitStructPtr->Speed = GPIO_SPEED_FAST;
	GPIO_InitStructPtr->Alternate = RGB_LED_GetLED_AF(LED_PORT, LED_PIN); //ex: PA5 AF is GPIO_AF1_TIM2;
	HAL_GPIO_Init(LED_PORT, GPIO_InitStructPtr);

}

/* ********************************************************************************
** FUNCTION NAME: RGB_LED_Start()
** DESCRIPTION:
** NOTE:		None.
*********************************************************************************** */
void RGB_LED_Start()
{
	//Make sure you called RGB_LED_Init before starting pwm (first time only)
	//Start RGB1 LED PWM generation
	RGB_LED_StartLED(&RGB1_RedTimHandle, RGB1_RED_TIM_CH);
	RGB_LED_StartLED(&RGB1_GreenTimHandle, RGB1_GREEN_TIM_CH);
	RGB_LED_StartLED(&RGB1_BlueTimHandle, RGB1_BLUE_TIM_CH);
}

/* ********************************************************************************
** FUNCTION NAME: RGB_LED_StartLED()
** DESCRIPTION: Initialize individual LED TIM Handler and PWM Config structures
** NOTE:		None.
*********************************************************************************** */
void RGB_LED_StartLED(TIM_HandleTypeDef * TimHandlePtr, uint32_t TimChannel){
	//Activate the TIM peripheral and start PWM generation output to one of the RGB LED color pins
	if(HAL_TIM_PWM_Start(TimHandlePtr, TimChannel) != HAL_OK)
		  {
		    /* Configuration Error */
		    RGB_LED_ErrorHandler();
		  }
}

/* ********************************************************************************
** FUNCTION NAME: RGB_LED_Stop()
** DESCRIPTION: Stop the PWM generation on all color lines of RGB LED
** NOTE:		None.
 *
*********************************************************************************** */
void RGB_LED_Stop(void)
{
	RGB_LED_StopLED(&RGB1_RedTimHandle, RGB1_RED_TIM_CH);
	RGB_LED_StopLED(&RGB1_GreenTimHandle, RGB1_GREEN_TIM_CH);
	RGB_LED_StopLED(&RGB1_BlueTimHandle, RGB1_BLUE_TIM_CH);
  return;
}

/* ********************************************************************************
** FUNCTION NAME: RGB_LED_StopLED()
** DESCRIPTION: Stop the PWM generation on single color line on RGB LED
** NOTE:		None.
 *
*********************************************************************************** */
void RGB_LED_StopLED(TIM_HandleTypeDef * TimHandlePtr, uint32_t TimChannel)
{
	if(HAL_TIM_PWM_Stop(TimHandlePtr, TimChannel) != HAL_OK)
	  {
	    /* Configuration Error */
	    RGB_LED_ErrorHandler();
	  }

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
*********************************************************************************** */
void RGB_LED_SetColor(uint16_t red, uint16_t green, uint16_t blue)
{

  return;
}

/* ********************************************************************************
** FUNCTION NAME: RGB_LED_ErrorHandler()
** DESCRIPTION: Handles Errors....
** NOTE:		None.
 *
*********************************************************************************** */
void RGB_LED_ErrorHandler(void)
{
	//TODO: Need to implement this method

  return;
}


/* ********************************************************************************
** FUNCTION NAME: HAL_TIM_PWM_MspInit()
** DESCRIPTION: Called by HAL_TIM_PWM_MspInit
** NOTE:		This function overrides the function declared in stm32f4xx_hal_tim.h
 *
*********************************************************************************** */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *TimHandle){
	//enable timer clocks and led gpio ports
	if(TimHandle->Channel == RGB1_RED_TIM_CH && TimHandle->Instance == RGB1_RED_TIM_REG){
		RGB1_RED_TIMCLK_EN();
		RGB_LED_InitLEDPwm(&GPIO_RGB1_RedInitStruct, RGB1_RED_PORT, RGB1_RED_PIN);
	}
	else if(TimHandle->Channel == RGB1_GREEN_TIM_CH && TimHandle->Instance == RGB1_GREEN_TIM_REG){
			RGB1_GREEN_TIMCLK_EN();
			RGB_LED_InitLEDPwm(&GPIO_RGB1_GreenInitStruct, RGB1_GREEN_PORT, RGB1_GREEN_PIN);
	}
	else if(TimHandle->Channel == RGB1_BLUE_TIM_CH && TimHandle->Instance == RGB1_BLUE_TIM_REG){
				RGB1_BLUE_TIMCLK_EN();
				RGB_LED_InitLEDPwm(&GPIO_RGB1_BlueInitStruct, RGB1_BLUE_PORT, RGB1_BLUE_PIN);
	}
	else{
		RGB_LED_ErrorHandler();
	}
}


/* ********************************************************************************
** FUNCTION NAME: RGB_LED_GetLED_AF()
** DESCRIPTION: Call to get the associated Alternate Function number used for a specific port / pin #
** NOTE:		Can add more control statement, just using the ones attached to pins we know we are going to use
 *
*********************************************************************************** */
uint8_t RGB_LED_GetLED_AF(GPIO_TypeDef * LED_PORT, uint16_t LED_PIN){
	uint8_t af_num = AF_ERROR;//error number 0xFF;
	if(LED_PORT == RGB1_RED_PORT && LED_PIN == RGB1_RED_PIN){
			af_num = RGB1_RED_AF;//Alternate function pin for GPIO Timer
	}
	else if(LED_PORT == RGB1_GREEN_PORT && LED_PIN == RGB1_GREEN_PIN){
		af_num = RGB1_GREEN_AF;
	}
	else if(LED_PORT == RGB1_BLUE_PORT && LED_PIN == RGB1_BLUE_PIN){
		af_num = RGB1_BLUE_AF;
	}
	else{
		RGB_LED_ErrorHandler();

	}
	//return 0xFF if nothing
	return af_num;
}




/* ********************************************************************************
** FUNCTION NAME: RGB_LED_UpdateDutyCycle()
** DESCRIPTION: Updates the Duty Cycle /  Pulse value of the pwm used in the timer capture / compare register
** 				This essentially changed the voltage that the LED pin sees
** 				Ex: if duty cycle is 50% then voltage seen by pin is 50% of VCC
** NOTE:		Might need to only allow this during certain interrupt periods ... blocking?
 *
*********************************************************************************** */
void RGB_LED_UpdateDutyCycle(uint32_t duty_cycle_percent, TIM_HandleTypeDef * TimHandlePtr, TIM_OC_InitTypeDef * PwmConfigPtr){
	//timer_freq = system clock / (prescaler + 1)
	//Period(Cycles) = Pwm Period * timer_freq = timer_freq / pwm freq  --- where Pwm period = 1 / pwm freq

	uint32_t timer_freq = HAL_RCC_GetSysClockFreq() / RGB_PRESCALER_DFLT; // not including the + 1 ...just because! I may have reasons
	uint32_t timer_period = timer_freq / RGB_PWMFREQ_DFLT;
	//TODO: validate the duty cycle is a unsigned int value between 0 and 100
	//update the duty cycle value in the pwm structure (this does not update the pwm, it's just so record our new duty cycle value)
	PwmConfigPtr->Pulse = duty_cycle_percent * timer_period / 100;  //Pulse(Cycles) = Duty_Cycle * Period(Cycles) / 100 ex: 50% * 16000 / 100 = 8000

	//Now update the duty cycle by updating the pulse value, the timer will update the duty cycle after it completes its most recent period capture compare
	//TIMx->CCR1 = OC_Config->Pulse
	TimHandlePtr->Instance->CCR1 = PwmConfigPtr->Pulse; //now we use the value that was recorded in the pwm structure

	//TODO: maybe return OK or ERROR boolean status? for example is duty cycle is not valid?

}
