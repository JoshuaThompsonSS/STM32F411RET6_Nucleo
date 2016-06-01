/* File Name          : rgb_led_interface.c
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
 *///Common colors
rgb_color_t RedColor = {RGB_SOLID_RED, 0, 0};
rgb_color_t BlueColor = {0, 0, RGB_SOLID_GREEN};
rgb_color_t GreenColor = {0, RGB_SOLID_BLUE, 0};
rgb_color_t WhiteColor = {RGB_SOLID_RED, RGB_SOLID_GREEN, RGB_SOLID_BLUE};
rgb_color_t BlackColor = {0, 0, 0};

/*
 ********  FUNCTION DECLARATION & DEFINTION ********
 */
void FUNCTIONAL_REG_LED_InitHandle(void){
	//rgb driver handler
	rgbHandle.init = FUNCTIONAL_RGB_LED_Init;
	rgbHandle.de_init = FUNCTIONAL_RGB_LED_DeInit;
	rgbHandle.start = FUNCTIONAL_RGB_LED_Start;
	rgbHandle.stop = FUNCTIONAL_RGB_LED_Stop;
	rgbHandle.reset = FUNCTIONAL_RGB_LED_Reset;
	rgbHandle.get_color_diff = FUNCTIONAL_RGB_LED_GetColorDiff;
	rgbHandle.get_color = FUNCTIONAL_RGB_LED_GetColor;
	rgbHandle.set_color = FUNCTIONAL_RGB_LED_SetColor;
	return;
}
void FUNCTIONAL_RGB_LED_Init(void){
	RGB_LED_Init(FUNC_RGB_LED_NUM);
	return;
}

void FUNCTIONAL_RGB_LED_Start(void)
{
	RGB_LED_Start(FUNC_RGB_LED_NUM);
	return;
}
void FUNCTIONAL_RGB_LED_Stop(void){
	RGB_LED_Stop(FUNC_RGB_LED_NUM);
	return;
}
void FUNCTIONAL_RGB_LED_DeInit(void){
	RGB_LED_DeInit(FUNC_RGB_LED_NUM);
	return;
}
void FUNCTIONAL_RGB_LED_Reset(void){
	RGB_LED_Reset(FUNC_RGB_LED_NUM);
	return;
}
void FUNCTIONAL_RGB_LED_SetColor(rgb_color_t * color){
	RGB_LED_SetColor(FUNC_RGB_LED_NUM, color);
	return;
}
void FUNCTIONAL_RGB_LED_GetColor(rgb_color_t * color){
	RGB_LED_GetColor(FUNC_RGB_LED_NUM, color);
	return;
}
void FUNCTIONAL_RGB_LED_GetColorDiff(rgb_color_t * color1, rgb_color_t * color2, rgb_color_t * colorDelta){
	colorDelta->red = color1->red - color2->red;
	colorDelta->green = color1->green - color2->green;
	colorDelta->blue = color1->blue - color2->blue;
	return;
}

//Generate step
void FUNCTIONAL_RGB_LED_InitRampStep(rgb_led_step_t * step){
	step->last_step = false;
	step->step_type = RAMP_STEP;
	step->complete = false;
	step->mode = RGB_LIN_MODE;
	step->func_handler = FUNCTIONAL_RGB_LED_Ramp;
	return;
}
void FUNCTIONAL_RGB_LED_InitHoldStep(rgb_led_step_t * step){
	step->last_step = false;
	step->step_type = HOLD_STEP;
	step->complete = false;
	step->mode = RGB_LIN_MODE;
	step->func_handler = FUNCTIONAL_RGB_LED_Hold;
	return;
}
void FUNCTIONAL_RGB_LED_InitSetpointStep(rgb_led_step_t * step){
	step->last_step = false;
	step->step_type = SETPOINT_STEP;
	step->complete = false;
	step->mode = RGB_NO_MODE;
	step->func_handler = FUNCTIONAL_RGB_LED_Setpoint;
	return;
}
void FUNCTIONAL_RGB_LED_InitRepeatStep(rgb_led_step_t * step){
	step->last_step = false;
	step->step_type = REPEAT_STEP;
	step->complete = false;
	step->mode = RGB_NO_MODE;
	step->func_handler = FUNCTIONAL_RGB_LED_Repeat;
	return;
}

//Init Sequences
void FUNCTIONAL_RGB_LED_InitOnSeq(rgb_led_sequence_t * sequence){
	//Setpoint Step - set solid color black (off)
	rgb_led_step_t setpoint_step1;
	setpoint_step1.current_step_num = 0;
	setpoint_step1.color_setpoint = BlackColor;
	setpoint_step1.step_type = SETPOINT_STEP;
	setpoint_step1.next_step_num = 1;
	setpoint_step1.last_step = false;
	setpoint_step1.func_handler = FUNCTIONAL_RGB_LED_Setpoint;
	setpoint_step1.complete = false;

	//Ramp Step - increase color to white at a duration of 1 sec
	rgb_led_step_t ramp_step2;
	ramp_step2.time = 0;
	ramp_step2.step_type = RAMP_STEP;
	ramp_step2.current_step_num = 1;
	ramp_step2.color_setpoint = WhiteColor;
	ramp_step2.duration = 1;
	ramp_step2.next_step_num = 2;
	ramp_step2.last_step = false;
	ramp_step2.func_handler = FUNCTIONAL_RGB_LED_Ramp;
	ramp_step2.complete = false;

	//Hold Step - set color to white and hold for duration of 1 sec
	//			- make this step the last step (so sequence repeats)
	rgb_led_step_t hold_step3;
	hold_step3.step_type = HOLD_STEP;
	hold_step3.time = 0;
	hold_step3.current_step_num = 2;
	hold_step3.color_setpoint = WhiteColor;
	hold_step3.duration = 1;
	hold_step3.next_step_num = 3; //go back to beginning
	hold_step3.last_step = false;
	hold_step3.func_handler = FUNCTIONAL_RGB_LED_Hold;
	hold_step3.complete = false;

	//Ramp Step - increase color to white at a duration of 1 sec
	rgb_led_step_t ramp_step4;
	ramp_step4.step_type = RAMP_STEP;
	ramp_step4.time = 0;
	ramp_step4.current_step_num = 3;
	ramp_step4.color_setpoint = BlackColor;
	ramp_step4.duration = 1;
	ramp_step4.next_step_num = 4;
	ramp_step4.last_step = false;
	ramp_step4.func_handler = FUNCTIONAL_RGB_LED_Ramp;
	ramp_step4.complete = false;

	//hold
	rgb_led_step_t hold_step5;
	hold_step5.step_type = HOLD_STEP;
	hold_step5.time = 0;
	hold_step5.current_step_num = 4;
	hold_step5.color_setpoint = BlackColor;
	hold_step5.duration = 1;
	hold_step5.next_step_num = 0; //go back to beginning
	hold_step5.last_step = true;
	hold_step5.func_handler = FUNCTIONAL_RGB_LED_Hold;
	hold_step5.complete = false;

	sequence->steps[0] = setpoint_step1;
	sequence->steps[1] = ramp_step2;
	sequence->steps[2] = hold_step3;
	sequence->steps[3] = ramp_step4;
	sequence->steps[4] = hold_step5;
	sequence->complete = false;
	sequence->current_step_num = 0;
	sequence->seq_type = ON;
	sequence->restart = true; //loop over and over

}
void FUNCTIONAL_RGB_LED_InitOffSeq(void){

}
void FUNCTIONAL_RGB_LED_InitCriticalSeq(void){

}
void FUNCTIONAL_RGB_LED_InitBTConnectedSeq(void){

}
void FUNCTIONAL_RGB_LED_InitBTDiscoverableSeq(void){

}
void FUNCTIONAL_RGB_LED_InitBTPairingSeq(void){

}
void FUNCTIONAL_RGB_LED_InitBTConnectingSeq(void){

}
void FUNCTIONAL_RGB_LED_InitAUXInShoeSeq(void){

}
void FUNCTIONAL_RGB_LED_InitAUXInShoe1Seq(void){

}
void FUNCTIONAL_RGB_LED_InitFWUpgradeSeq(void){

}
void FUNCTIONAL_RGB_LED_InitFWUpgradeCompleteSeq(void){

}



//Step functions to create specific RGB LED sequences

/* ********************************************************************************
** FUNCTION NAME: FUNCTIONAL_RGB_LED_Ramp()
** DESCRIPTION:
** 				- calculate color scale using diff between current color and setpoint and then divide by user define duration to reach setpoint
** 				- increment step time elapsed
** 				- see if step color reached setpoint and duration == current time and if so then step is complete
** 				- if color has not reached setpoint yet then increment pwm output to rgb led driver by using scale and time increment
** NOTE:
 *
*********************************************************************************** */
void FUNCTIONAL_RGB_LED_Ramp(rgb_led_step_t * step){
	RGB_LED_GetColor(FUNC_RGB_LED_NUM, &step->color);
	rgb_color_t dlta_clr;
	FUNCTIONAL_RGB_LED_GetColorDiff(&step->color_setpoint, &step->color, &dlta_clr);
	if(step->time <= 0){
		step->scales.red = dlta_clr.red / step->duration;
		step->scales.green = dlta_clr.green / step->duration;
		step->scales.blue = dlta_clr.blue / step->duration;
	}
	step->time += STEP_TIME_PER_CYCLE; //time for each step interrupt loop
	int color_diff = dlta_clr.red + dlta_clr.green + dlta_clr.blue;
	if(color_diff < 0){color_diff = color_diff * (-1);} //abs value

	if((color_diff <= MIN_COLOR_DIFF) && (step->time >= step->duration)){
		step->complete = true;
		step->time = 0;
		return;
	}
	else{
		//increment color and update rgb led driver pwm -- keep doing this until color setpoit reached
		step->color.red += STEP_TIME_PER_CYCLE * step->scales.red;
		step->color.green += STEP_TIME_PER_CYCLE * step->scales.green;
		step->color.blue += STEP_TIME_PER_CYCLE * step->scales.blue;
		FUNCTIONAL_RGB_LED_SetColor(&step->color);
	}

	return;
}

/* ********************************************************************************
** FUNCTION NAME: FUNCTIONAL_RGB_LED_Hold()
** DESCRIPTION:
** 				- set color to hold
** 				- increment step time elapsed
** 				- see if step duration == current time then step is complete
** NOTE:
 *
*********************************************************************************** */
void FUNCTIONAL_RGB_LED_Hold(rgb_led_step_t * step){
	if(step->time <= 0){FUNCTIONAL_RGB_LED_SetColor(&step->color_setpoint);}
	step->time += STEP_TIME_PER_CYCLE; //time for each step interrupt loop
	if(step->time >= step->duration){
		step->complete = true;
		step->time = 0;
	}
	return;
}

/* ********************************************************************************
** FUNCTION NAME: FUNCTIONAL_RGB_LED_Setpoint()
** DESCRIPTION:
** 				- set color to setpoint and that's it
** NOTE:
 *
*********************************************************************************** */
void FUNCTIONAL_RGB_LED_Setpoint(rgb_led_step_t * step){
	FUNCTIONAL_RGB_LED_SetColor(&step->color_setpoint);
	step->complete = true;
	return;
}

/* ********************************************************************************
** FUNCTION NAME: FUNCTIONAL_RGB_LED_Repeat()
** DESCRIPTION:
** 				- point parent sequence to starting step
** 				- repeat until repeat counts == duration (counts) set by user
** NOTE:
 *
*********************************************************************************** */
void FUNCTIONAL_RGB_LED_Repeat(rgb_led_step_t * step){
	if(step->time >= step->duration){
		step->complete = true;
		return;
	}
	//check to see if step has a parent sequence
	if(step->parent_seq == NULL){
		return;
	}
	//move parent sequence step index to starting step in repeat loop
	step->parent_seq->current_step_num = step->next_step_num;
	return;
}

//Sequence routine service - should be called at least every 100 ms by SysTick_Handler in stm32f4xx_it.c
void FUNCTIONAL_RGB_LED_IRQHandler(void){
	return;
}


