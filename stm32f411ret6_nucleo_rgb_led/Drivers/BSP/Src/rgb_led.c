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
 * 1. Initialize Config Structure, RGB LED TIMER, PWM and GPIO Registers / Peripherals
 *    - RGB_LED_Init(rgbnum); //rgbnum is the RGB LED you are using (0 is functional rgb led and 1 is decorative rgb led)
 * 2. Start the RGB LED PWM generator - this generates three pwm outputs for the three individual leds of the rgb led
 *    - RGB_LED_Start(rgbnum); //rgbnum is the RGB LED number you are using
 *
 *Normal Operation
 * 1. Set the RGB color of the LED by setting the three pwm outputs (0 - 100%) of the RGB LED pins, this will update the voltages seen by the RGB LED
 *    RGB_LED_SetColor(rggnum, red, blue, green); //where red, blue, green is a number 0 - 255
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
#include "rgb_led.h"


/*
 ********  DEFINE CONSTANTS & DATA TYPE (GLOBAL VARIABLES) ********
 */

//hold collection of rgd led configs -- since we are going to use two or possibly more rgb leds


/*
 ********  FUNCTION DECLARATION & DEFINTION ********
 */


/* ******************************************************************
** FUNCTION NAME: RGB_LED_InitConfigs()
 * DESCRIPTION: Initialize all RGB Config Structures
 * NOTE: Initialize the RGB_LED Structures
 *********************************************************************************** */
void RGB_LED_InitConfigs(int rgbnum)
{
	if(rgbnum >= RGB_LED_COUNT){
		RGB_LED_ErrorHandler();
		return;
	}

	//GPIO Port Config
	RgbLedConfigs[rgbnum].red.port = RGB1_RED_PORT;
	RgbLedConfigs[rgbnum].green.port = RGB1_GREEN_PORT;
	RgbLedConfigs[rgbnum].blue.port = RGB1_BLUE_PORT;

	//GPIO Pin
	RgbLedConfigs[rgbnum].red.pin = RGB1_RED_PIN;
	RgbLedConfigs[rgbnum].green.pin = RGB1_GREEN_PIN;
	RgbLedConfigs[rgbnum].blue.pin = RGB1_BLUE_PIN;

	//TIM Channel
	RgbLedConfigs[rgbnum].red.timChannel = RGB1_RED_TIM_CH;
	RgbLedConfigs[rgbnum].green.timChannel = RGB1_GREEN_TIM_CH;
	RgbLedConfigs[rgbnum].blue.timChannel = RGB1_BLUE_TIM_CH;

	//TIMx Reg
	RgbLedConfigs[rgbnum].red.timReg = RGB1_RED_TIM_REG;
	RgbLedConfigs[rgbnum].green.timReg = RGB1_GREEN_TIM_REG;
	RgbLedConfigs[rgbnum].blue.timReg = RGB1_BLUE_TIM_REG;

	//Alternate Function Num
	RgbLedConfigs[rgbnum].red.afNum = RGB1_RED_AF;
	RgbLedConfigs[rgbnum].green.afNum = RGB1_GREEN_AF;
	RgbLedConfigs[rgbnum].blue.afNum = RGB1_BLUE_AF;


}

/* ******************************************************************
** FUNCTION NAME: RGB_LED_Init()
 * DESCRIPTION: Initialize all RGB Led structures / timers
 * NOTE: Initialize the RGB_LED
 *********************************************************************************** */
void RGB_LED_Init(int rgbnum)
{
	if(rgbnum >= RGB_LED_COUNT){
		RGB_LED_ErrorHandler();
		return;
	}
	RGB_LED_InitConfigs(rgbnum);

	//TODO: add ability to init different RGB LEDs (right now just initializing RGB LED1 manually)
	//Maybe add argument to this function to init specified RGB LED..?

	//Init RGB LEDx Handler and Config

	RGB_LED_InitPWMConfig(&RgbLedConfigs[rgbnum].red);
	RGB_LED_InitPWMConfig(&RgbLedConfigs[rgbnum].green);
	RGB_LED_InitPWMConfig(&RgbLedConfigs[rgbnum].blue);

	//Once initialized, then ok to call RGB_LED_Start

  return;
}

/* ********************************************************************************
** FUNCTION NAME: RGB_LED_DeInit()
** DESCRIPTION: RGB_LED to de-initialize the driver component.
** NOTE:		None.
*********************************************************************************** */
void RGB_LED_DeInit(int rgbnum)
{
  /* Deinitialize RGB LED interface - pwm, clocks and gpio */
	if(rgbnum >= RGB_LED_COUNT){
			RGB_LED_ErrorHandler();
			return;
	}
	RGB_LED_InitConfigs(rgbnum);

	//TODO: add ability to init different RGB LEDs (right now just initializing RGB LED1 manually)
	//Maybe add argument to this function to init specified RGB LED..?

	//Init RGB LEDx Handler and Config

	RGB_LED_DeInitPWMConfig(&RgbLedConfigs[rgbnum].red);
	RGB_LED_DeInitPWMConfig(&RgbLedConfigs[rgbnum].green);
	RGB_LED_DeInitPWMConfig(&RgbLedConfigs[rgbnum].blue);

	//Once initialized, then ok to call RGB_LED_Start

}

/* ********************************************************************************
** FUNCTION NAME: RGB_LED_Reset()
** DESCRIPTION: Reset RGBx LED - stop pwm, configure data struct, initialize
** NOTE:		Does not call the RGB_LED_Start method - need to call after reset
*********************************************************************************** */
void RGB_LED_Reset(int rgbnum)
{
	if(rgbnum >= RGB_LED_COUNT){
				RGB_LED_ErrorHandler();
				return;
	}
	/* Stop any currently running pwm leds */
	RGB_LED_Stop(rgbnum);
	/* Deinitialize pwm */
	RGB_LED_DeInit(rgbnum);
	/* Initialize */
	RGB_LED_Init(rgbnum);

	/* TODO: */

  return;
}


/* ********************************************************************************
** FUNCTION NAME: RGB_LED_InitLEDConfig()
** DESCRIPTION: Initialize individual LED TIM Handler and PWM Config structures
** NOTE:		None.
*********************************************************************************** */

void RGB_LED_InitPWMConfig(rgb_led_color_conf_t * colorConfig){

	colorConfig->timHandle.Instance = colorConfig->timReg;
	colorConfig->timHandle.Channel = colorConfig->timChannel;
	//timer_freq = system clock / (prescaler + 1)
	//Period(Cycles) = Pwm Period * timer_freq = timer_freq / pwm freq  --- where Pwm period = 1 / pwm freq
	uint32_t timer_freq = HAL_RCC_GetSysClockFreq() / RGB_PRESCALER_DFLT; // not including the + 1 ...just because! I may have reasons
	uint32_t timer_period = timer_freq / RGB_PWMFREQ_DFLT;
	colorConfig->timHandle.Init.Period = timer_period;
	colorConfig->timHandle.Init.Prescaler = RGB_PRESCALER_DFLT;
	//pwm freq = timer period / timer_freq = 1000 Hz (or whatever is defined by RGB_PWMFREQ_DFLT)
	colorConfig->timHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	colorConfig->timHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	colorConfig->timHandle.Init.RepetitionCounter = 0;

	//configure TIM mode for pwm function generator
	//HAL_TIM_PWM_MspInit(&TimHandle);
	if(HAL_TIM_PWM_Init(&colorConfig->timHandle) != HAL_OK)
	{
		/* Configuration Error */
		RGB_LED_ErrorHandler();
	}

	colorConfig->pwmConfig.OCMode = TIM_OCMODE_PWM2; //PWM1
	colorConfig->pwmConfig.OCFastMode = TIM_OCFAST_DISABLE;
	colorConfig->pwmConfig.OCPolarity = TIM_OCPOLARITY_HIGH; //HIGH
	colorConfig->pwmConfig.Pulse = RGB_PWMDUTY_DFLT * timer_period / 100;  //Pulse(Cycles) = Duty_Cycle * Period(Cycles) / 100 ex: 50% * 16000 / 100 = 8000
	if(HAL_TIM_PWM_ConfigChannel(&colorConfig->timHandle, &colorConfig->pwmConfig, colorConfig->timChannel) != HAL_OK)
	{
		/* Configuration Error */
		RGB_LED_ErrorHandler();
	}
}

/* ********************************************************************************
** FUNCTION NAME: RGB_LED_DeInitLEDConfig()
** DESCRIPTION: De-Initialize individual LED TIM Handler and PWM Config structures
** NOTE:		None.
*********************************************************************************** */
void RGB_LED_DeInitPWMConfig(rgb_led_color_conf_t * colorConfig){

	colorConfig->timHandle.Instance = colorConfig->timReg;
	//De-Initialize TIM / PWM mode
	//HAL_TIM_PWM_MspInit(&TimHandle);
	if(HAL_TIM_PWM_DeInit(&colorConfig->timHandle) != HAL_OK)
	{
		/* Configuration Error */
		RGB_LED_ErrorHandler();
	}

}

/* ********************************************************************************
** FUNCTION NAME: RRGB_LED_InitLEDGpio()
** DESCRIPTION:
** NOTE:		This method is called by the HAL_TIM_PWM_MspInit callback
*********************************************************************************** */
void RGB_LED_InitLEDGpio(rgb_led_color_conf_t * colorConfig){

	//Enable GPIOx peripheral clock
	RGB_LED_EnGpioClk(colorConfig->port);

	//Enable GPIOx pin as output and assign pin number
	colorConfig->gpioInitStruct.Pin = colorConfig->pin;
	colorConfig->gpioInitStruct.Mode = GPIO_MODE_AF_PP;
	colorConfig->gpioInitStruct.Pull = GPIO_PULLUP;//GPIO_PULLDOWN; NOPULL
	colorConfig->gpioInitStruct.Speed = GPIO_SPEED_LOW;
	colorConfig->gpioInitStruct.Alternate = colorConfig->afNum; //ex: PA5 AF is GPIO_AF1_TIM2;
	HAL_GPIO_Init(colorConfig->port, &colorConfig->gpioInitStruct);

}

/* ********************************************************************************
** FUNCTION NAME: RRGB_LED_DeInitLEDGpio()
** DESCRIPTION: Deinitialize gpio ports to default
** NOTE:		This method is called by the HAL_TIM_PWM_MspInit callback
*********************************************************************************** */
void RGB_LED_DeInitLEDGpio(rgb_led_color_conf_t * colorConfig){

	//Enable GPIOx peripheral clock
	RGB_LED_EnGpioClk(colorConfig->port);

	//Enable GPIOx pin as output and assign pin number
	colorConfig->gpioInitStruct.Pin = colorConfig->pin;
	colorConfig->gpioInitStruct.Mode = GPIO_MODE_AF_PP;
	colorConfig->gpioInitStruct.Pull = GPIO_NOPULL;//GPIO_PULLDOWN;
	colorConfig->gpioInitStruct.Speed = GPIO_SPEED_LOW;
	colorConfig->gpioInitStruct.Alternate = colorConfig->afNum; //ex: PA5 AF is GPIO_AF1_TIM2;
	HAL_GPIO_DeInit(colorConfig->port, colorConfig->gpioInitStruct.Pin);

}

/* ********************************************************************************
** FUNCTION NAME: RGB_LED_Start()
** DESCRIPTION:
** NOTE:		None.
*********************************************************************************** */
void RGB_LED_Start(int rgbnum)
{
	if(rgbnum >= RGB_LED_COUNT){
		RGB_LED_ErrorHandler();
		return;
	}
	//Make sure you called RGB_LED_Init before starting pwm (first time only)
	//Start RGB LEDx PWM generation
	RGB_LED_StartLED(&RgbLedConfigs[rgbnum].red);
	RGB_LED_StartLED(&RgbLedConfigs[rgbnum].green);
	RGB_LED_StartLED(&RgbLedConfigs[rgbnum].blue);
}

/* ********************************************************************************
** FUNCTION NAME: RGB_LED_StartLED()
** DESCRIPTION: Initialize individual LED TIM Handler and PWM Config structures
** NOTE:		None.
*********************************************************************************** */
void RGB_LED_StartLED(rgb_led_color_conf_t * colorConfig){
	//Activate the TIM peripheral and start PWM generation output to one of the RGB LED color pins
	if(HAL_TIM_PWM_Start(&colorConfig->timHandle, colorConfig->timChannel) != HAL_OK)
		  {
		    /* Configuration Error */
		    RGB_LED_ErrorHandler();
		  }
}

/* ********************************************************************************
** FUNCTION NAME: RGB_LED_Stop()
** DESCRIPTION: Stop the PWM generation on all color lines of RGBx LED
** NOTE:		None.
 *
*********************************************************************************** */
void RGB_LED_Stop(int rgbnum)
{
	if(rgbnum >= RGB_LED_COUNT){
			RGB_LED_ErrorHandler();
			return;
	}

	RGB_LED_StopLED(&RgbLedConfigs[rgbnum].red);
	RGB_LED_StopLED(&RgbLedConfigs[rgbnum].green);
	RGB_LED_StopLED(&RgbLedConfigs[rgbnum].blue);
  return;
}

/* ********************************************************************************
** FUNCTION NAME: RGB_LED_StopLED()
** DESCRIPTION: Stop the PWM generation on single color line on RGB LED
** NOTE:		None.
 *
*********************************************************************************** */
void RGB_LED_StopLED(rgb_led_color_conf_t * colorConfig)
{
	if(HAL_TIM_PWM_Stop(&colorConfig->timHandle, colorConfig->timChannel) != HAL_OK)
	  {
	    /* Configuration Error */
	    RGB_LED_ErrorHandler();
	  }

  return;
}


/* ********************************************************************************
** FUNCTION NAME: RGB_LED_PowerDown()
** DESCRIPTION: This power downs the driver
** NOTE:		None.
 *Power-Down Sequence:
*********************************************************************************** */
void RGB_LED_PowerDown(int rgbnum)
{
	if(rgbnum >= RGB_LED_COUNT){
				RGB_LED_ErrorHandler();
				return;
		}
	//TODO: what else besides calling RGB_LED_Stop? This just stops pwm, what else should be power down? GPIOs?
	RGB_LED_Stop(rgbnum);

  return;
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
** DESCRIPTION: set the three color pins pwm duty cycle of the rgb led to change color
** NOTE:		color variable is struct that just defines three int representing red, green and blue - range (0 - 255)
 *
*********************************************************************************** */
void RGB_LED_SetColor(int rgbnum, rgb_color_t * color)
{
	if(rgbnum >= RGB_LED_COUNT){
		RGB_LED_ErrorHandler();
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

	//update RED pin of RGB LED - get the pwm duty cycle % equivalent of the 0 - 255 color value
	float duty_cycle = RGB_PWM_OFFSET_RED + (color->red * RGB_TO_PWM_SCALE_RED); //ex: 240 * (100% / 255) = 94.1 % duty cycle (brightness %)
	RGB_LED_SetDutyCycle(duty_cycle, &RgbLedConfigs[rgbnum].red);

	//GREEN
	duty_cycle = RGB_PWM_OFFSET_GREEN + (color->green * RGB_TO_PWM_SCALE_GREEN);
	RGB_LED_SetDutyCycle(duty_cycle, &RgbLedConfigs[rgbnum].green);

	//BLUE
	duty_cycle = RGB_PWM_OFFSET_BLUE + (color->blue * RGB_TO_PWM_SCALE_BLUE);
	RGB_LED_SetDutyCycle(duty_cycle, &RgbLedConfigs[rgbnum].blue);


  return;
}

/* ********************************************************************************
** FUNCTION NAME: RGB_LED_GetColor()
** DESCRIPTION: get the current color settings of the rgb led
** NOTE:		color variable is struct that just defines three int representing red, green and blue - range (0 - 255)
 *
*********************************************************************************** */
void RGB_LED_GetColor(int rgbnum, rgb_color_t * color)
{
	if(rgbnum >= RGB_LED_COUNT){
		RGB_LED_ErrorHandler();
		return;
	}
	//RED - get red color rgb value 0 - 255
	float duty_cycle=0.0;
	duty_cycle = RGB_LED_GetDutyCycle(&RgbLedConfigs[rgbnum].red);
	color->red = (duty_cycle - RGB_PWM_OFFSET_RED) / RGB_TO_PWM_SCALE_RED;

	//GREEN
	duty_cycle = RGB_LED_GetDutyCycle(&RgbLedConfigs[rgbnum].green);
	color->green = (duty_cycle - RGB_PWM_OFFSET_GREEN) / RGB_TO_PWM_SCALE_GREEN;

	//BLUE
	duty_cycle = RGB_LED_GetDutyCycle(&RgbLedConfigs[rgbnum].blue);
	color->blue = (duty_cycle - RGB_PWM_OFFSET_BLUE) / RGB_TO_PWM_SCALE_BLUE;


  return;
}


/* ********************************************************************************
** FUNCTION NAME: RGB_LED_SetDutyCycle()
** DESCRIPTION: Updates the Duty Cycle /  Pulse value of the pwm used in the timer capture / compare register
** 				This essentially changed the voltage that the LED pin sees therefore changing the color of the entire rgb led
** 				Ex: if duty cycle is 50% then voltage seen by pin is 50% of VCC
** NOTE:		Might need to only allow this during certain interrupt periods ... blocking?
** 			    RGB_LED_SetColor calls this function
 *
*********************************************************************************** */
void RGB_LED_SetDutyCycle(float duty_cycle_percent, rgb_led_color_conf_t * colorConfig){
	//timer_freq = system clock / (prescaler + 1)
	//Period(Cycles) = Pwm Period * timer_freq = timer_freq / pwm freq  --- where Pwm period = 1 / pwm freq

	uint32_t timer_freq = HAL_RCC_GetSysClockFreq() / RGB_PRESCALER_DFLT; // not including the + 1 ...just because! I may have reasons
	uint32_t timer_period = timer_freq / RGB_PWMFREQ_DFLT;
	//TODO: validate the duty cycle is a unsigned int value between 0 and 100
	//update the duty cycle value in the pwm structure (this does not update the pwm, it's just so record our new duty cycle value)
	colorConfig->pwmConfig.Pulse = duty_cycle_percent * timer_period / 100;  //Pulse(Cycles) = Duty_Cycle * Period(Cycles) / 100 ex: 50% * 16000 / 100 = 8000

	//Now update the duty cycle by updating the pulse value, the timer will update the duty cycle after it completes its most recent period capture compare
	//TIMx->CCR1 = OC_Config->Pulse
	if(colorConfig->timChannel==TIM_CHANNEL_1){
		colorConfig->timHandle.Instance->CCR1 = colorConfig->pwmConfig.Pulse; //now we use the value that was recorded in the pwm structure
	}
	else if(colorConfig->timChannel==TIM_CHANNEL_2){
		colorConfig->timHandle.Instance->CCR2 = colorConfig->pwmConfig.Pulse; //now we use the value that was recorded in the pwm structure
	}
	else if(colorConfig->timChannel==TIM_CHANNEL_3){
		colorConfig->timHandle.Instance->CCR3 = colorConfig->pwmConfig.Pulse; //now we use the value that was recorded in the pwm structure
	}
	else if(colorConfig->timChannel==TIM_CHANNEL_4){
		colorConfig->timHandle.Instance->CCR4 = colorConfig->pwmConfig.Pulse; //now we use the value that was recorded in the pwm structure
	}

	//TODO: maybe return OK or ERROR boolean status? for example is duty cycle is not valid?

}

/* ********************************************************************************
** FUNCTION NAME: RGB_LED_GetDutyCycle()
** DESCRIPTION: Read the Duty Cycle /  Pulse value of the pwm used for a single color on the RGB LED
**
** NOTE:		RGB_LED_GetColor calls this function
 *
*********************************************************************************** */
float RGB_LED_GetDutyCycle(rgb_led_color_conf_t * colorConfig){

	uint32_t timer_freq = HAL_RCC_GetSysClockFreq() / RGB_PRESCALER_DFLT; // not including the + 1 ...just because! I may have reasons
	uint32_t timer_period = timer_freq / RGB_PWMFREQ_DFLT;

	uint32_t duty_cycle_pulses;
	float duty_cycle_percent;
	//Now update the duty cycle by updating the pulse value, the timer will update the duty cycle after it completes its most recent period capture compare
	//TIMx->CCR1 = OC_Config->Pulse
	if(colorConfig->timChannel==TIM_CHANNEL_1){
		duty_cycle_pulses = colorConfig->pwmConfig.Pulse; //this is the PULSE value that is essentially the duty cycle in clocks of the pwm
	}
	else if(colorConfig->timChannel==TIM_CHANNEL_2){
		duty_cycle_pulses =colorConfig->pwmConfig.Pulse;;
	}
	else if(colorConfig->timChannel==TIM_CHANNEL_3){
		duty_cycle_pulses = colorConfig->pwmConfig.Pulse;;
	}
	else if(colorConfig->timChannel==TIM_CHANNEL_4){
		duty_cycle_pulses = colorConfig->pwmConfig.Pulse;;
	}

	duty_cycle_percent = duty_cycle_pulses * 100.0;
	duty_cycle_percent = duty_cycle_percent / timer_period;
	return duty_cycle_percent;

	//TODO: maybe return OK or ERROR boolean status? for example is duty cycle is not valid?

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
** FUNCTION NAME: RGB_LED_EnTimClk()
** DESCRIPTION: Enable timer clock
** NOTE:
 *
*********************************************************************************** */
void RGB_LED_EnTimClk(TIM_TypeDef * TimRegx){
	if(TimRegx==TIM1){
		__TIM1_CLK_ENABLE();
		return;
	}
	else if(TimRegx==TIM2){
		__TIM2_CLK_ENABLE();
		return;
	}
	else if(TimRegx==TIM3){
		__TIM3_CLK_ENABLE();
		return;
	}
	else if(TimRegx==TIM4){
		__TIM4_CLK_ENABLE();
		return;
	}
	else{
		RGB_LED_ErrorHandler();
		return;
	}
}

/* ********************************************************************************
** FUNCTION NAME: RGB_LED_DisTimClk()
** DESCRIPTION: Disable timer clock
** NOTE:
 *
*********************************************************************************** */
void RGB_LED_DisTimClk(TIM_TypeDef * TimRegx){
	if(TimRegx==TIM1){
		__TIM1_CLK_DISABLE();
		return;
	}
	else if(TimRegx==TIM2){
		__TIM2_CLK_DISABLE();
		return;
	}
	else if(TimRegx==TIM3){
		__TIM3_CLK_DISABLE();
		return;
	}
	else if(TimRegx==TIM4){
		__TIM4_CLK_DISABLE();
		return;
	}
	else{
		RGB_LED_ErrorHandler();
		return;
	}
}



/* ********************************************************************************
** FUNCTION NAME: RGB_LED_EnGpioClk()
** DESCRIPTION: Enable the correct GPIO Clock based on port argument
** NOTE:
 *
*********************************************************************************** */
void RGB_LED_EnGpioClk(GPIO_TypeDef * port){
	if(port == GPIOA){
		__HAL_RCC_GPIOA_CLK_ENABLE();
	}
	else if(port == GPIOB){
		__HAL_RCC_GPIOB_CLK_ENABLE();
	}
	else if(port == GPIOC){
		__HAL_RCC_GPIOC_CLK_ENABLE();
	}
	else if(port == GPIOD){
			__HAL_RCC_GPIOD_CLK_ENABLE();
		}
	else if(port == GPIOE){
			__HAL_RCC_GPIOE_CLK_ENABLE();
		}
	else{
		//GPIOH is not available for Timer based PWM generation
		RGB_LED_ErrorHandler();
	}
}


/* ********************************************************************************
** FUNCTION NAME: HAL_TIM_PWM_MspInit()
** DESCRIPTION: HAL_TIM_PWM_Init
** NOTE:		This function overrides the function declared in stm32f4xx_hal_tim.h
 *
*********************************************************************************** */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *TimHandle){
	//enable timer clocks and led gpio ports
	for(int i = 0; i<RGB_LED_COUNT; i++){
		if(TimHandle->Channel == RgbLedConfigs[i].red.timChannel && TimHandle->Instance == RgbLedConfigs[i].red.timReg){
			RGB_LED_EnTimClk(RgbLedConfigs[i].red.timReg);
			RGB_LED_InitLEDGpio(&RgbLedConfigs[i].red);
			return;
		}
		else if(TimHandle->Channel == RgbLedConfigs[i].green.timChannel && TimHandle->Instance == RgbLedConfigs[i].green.timReg){
			RGB_LED_EnTimClk(RgbLedConfigs[i].green.timReg);
			RGB_LED_InitLEDGpio(&RgbLedConfigs[i].green);
			return;
		}
		else if(TimHandle->Channel == RgbLedConfigs[i].blue.timChannel && TimHandle->Instance == RgbLedConfigs[i].blue.timReg){
			RGB_LED_EnTimClk(RgbLedConfigs[i].blue.timReg);
			RGB_LED_InitLEDGpio(&RgbLedConfigs[i].blue);
			return;
		}
		else{
			//TODO: ?
		}
	}
	RGB_LED_ErrorHandler();
}

/* ********************************************************************************
** FUNCTION NAME: HAL_TIM_PWM_MspDeInit()
** DESCRIPTION: HAL_TIM_PWM_MspDeInit
** NOTE:		This function overrides the function declared in stm32f4xx_hal_tim.h
 *
*********************************************************************************** */
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *TimHandle){
	//disable timer clocks and led gpio ports
		for(int i = 0; i<RGB_LED_COUNT; i++){
			if(TimHandle->Channel == RgbLedConfigs[i].red.timChannel && TimHandle->Instance == RgbLedConfigs[i].red.timReg){
				RGB_LED_DisTimClk(RgbLedConfigs[i].red.timReg);
				RGB_LED_DeInitLEDGpio(&RgbLedConfigs[i].red);
				return;
			}
			else if(TimHandle->Channel == RgbLedConfigs[i].green.timChannel && TimHandle->Instance == RgbLedConfigs[i].green.timReg){
				RGB_LED_DisTimClk(RgbLedConfigs[i].green.timReg);
				RGB_LED_DeInitLEDGpio(&RgbLedConfigs[i].green);
				return;
			}
			else if(TimHandle->Channel == RgbLedConfigs[i].blue.timChannel && TimHandle->Instance == RgbLedConfigs[i].blue.timReg){
				RGB_LED_DisTimClk(RgbLedConfigs[i].blue.timReg);
				RGB_LED_DeInitLEDGpio(&RgbLedConfigs[i].blue);
				return;
			}
			else{
				//TODO: ?
			}
		}
		RGB_LED_ErrorHandler();

}
