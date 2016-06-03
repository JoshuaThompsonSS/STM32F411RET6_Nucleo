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


/* ********************************************************************************
** FUNCTION NAME: FUNCTIONAL_RGB_LED_LoadSequence()
** DESCRIPTION:
** 				- Load a sequence into the rgbHandle sequence ptr but make sure to stop currently running seq
** NOTE:
 *
*********************************************************************************** */
void FUNCTIONAL_RGB_LED_LoadSequence(rgb_seq_type_t seqType){
	if(rgbHandle.sequence != NULL){
		rgbHandle.sequence->enabled = false; //disable currently running seq
		if(seqType < RGB_SEQ_COUNT && sequenceList[seqType]->valid){
			rgbHandle.next_sequence = sequenceList[seqType];
			rgbHandle.next_sequence->enabled = true;
		}
		else{
			FUNCTIONAL_RGB_LED_ErrorHandler();
		}
	}
	else{
		if(seqType < RGB_SEQ_COUNT && sequenceList[seqType]->valid){
					rgbHandle.sequence = sequenceList[seqType];
					rgbHandle.sequence->enabled = true;
					sequenceInitFuncList[rgbHandle.sequence->seq_type](rgbHandle.sequence);
				}
		else{
			FUNCTIONAL_RGB_LED_ErrorHandler();
		}
	}
}

/* ********************************************************************************
** FUNCTION NAME: FUNCTIONAL_RGB_LED_isRunning()
** DESCRIPTION:
** 				- Handle those errors!
** NOTE:
 *
*********************************************************************************** */
bool FUNCTIONAL_RGB_LED_isRunning(void){
	//TODO: handle stuff
	return funcRgbStatus.running;
}

/* ********************************************************************************
** FUNCTION NAME: FUNCTIONAL_RGB_LED_ErrorHandler()
** DESCRIPTION:
** 				- Handle those errors!
** NOTE:
 *
*********************************************************************************** */
void FUNCTIONAL_RGB_LED_ErrorHandler(void){
	//TODO: handle stuff
	funcRgbStatus.error = true;
}


/* ********************************************************************************
** FUNCTION NAME: FUNCTIONAL_RGB_LED_StartService()
** DESCRIPTION:
** 				-Initialize driver, interrupt routine and structures and then start the rgb led sequence interrupt driven routine
** NOTE:
 *
*********************************************************************************** */
void FUNCTIONAL_RGB_LED_StartService(void){
	//init rgb led driver handle (rgbHandle)
	FUNCTIONAL_RGB_LED_InitHandle();
	//init all the common functional rgb led sequences
	FUNCTIONAL_RGB_LED_InitSequences();
	//init driver
	rgbHandle.init();
	//start driver
	rgbHandle.start();

	//init timer interrupt
	FUNCTIONAL_RGB_LED_InitInterruptTimer();
	//start timer interrupt
	FUNCTIONAL_RGB_LED_StartInterruptTimer();
	funcRgbStatus.running = true;

	return;
}

/* ********************************************************************************
** FUNCTION NAME: FUNCTIONAL_RGB_LED_StopService()
** DESCRIPTION:
** 				-DeInitialize driver, de-initialize interrupt routine and then stop the rgb led sequence interrupt driven routine
** NOTE:
 *
*********************************************************************************** */
void FUNCTIONAL_RGB_LED_StopService(void){
	//start driver
	rgbHandle.stop();

	//init driver
	rgbHandle.de_init();

	if(rgbHandle.sequence != NULL){
		rgbHandle.sequence->enabled = false;
		rgbHandle.sequence = NULL;
	}

	//stop timer interrupt
	FUNCTIONAL_RGB_LED_StopInterruptTimer();

	//de-init timer interrupt
	FUNCTIONAL_RGB_LED_DeInitInterruptTimer();
	funcRgbStatus.running = false;


	return;
}

/* ********************************************************************************
** FUNCTION NAME: FUNCTIONAL_RGB_LED_InitHandle()
** DESCRIPTION:
** 				-Initialize the rgb led driver handle that provides easy access to common rgb led driver functions
** NOTE:
 *
*********************************************************************************** */
void FUNCTIONAL_RGB_LED_InitHandle(void){
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

/* ********************************************************************************
** FUNCTION NAME: FUNCTIONAL_RGB_LED_InitSequences()
** DESCRIPTION:
** 				-Initialize the common functional rgb led sequences
** NOTE:
 *
*********************************************************************************** */
void FUNCTIONAL_RGB_LED_InitSequences(void){
	//rgb led sequences commonly used
	int seqNum = 0;
	FUNCTIONAL_RGB_LED_InitOnSeq(&onSequence);
	sequenceList[seqNum] = &onSequence;
	sequenceInitFuncList[seqNum++] = FUNCTIONAL_RGB_LED_InitOnSeq;

	FUNCTIONAL_RGB_LED_InitOffSeq(&offSequence);
	sequenceList[seqNum] = &offSequence;
	sequenceInitFuncList[seqNum++] = FUNCTIONAL_RGB_LED_InitOffSeq;

	FUNCTIONAL_RGB_LED_InitCriticalSeq(&criticalBattSequence);
	sequenceList[seqNum] = &criticalBattSequence;
	sequenceInitFuncList[seqNum++] = FUNCTIONAL_RGB_LED_InitCriticalSeq;

	FUNCTIONAL_RGB_LED_InitBTConnectedSeq(&btConnectedSequence);
	sequenceList[seqNum] = &btConnectedSequence;
	sequenceInitFuncList[seqNum++] = FUNCTIONAL_RGB_LED_InitBTConnectedSeq;

	FUNCTIONAL_RGB_LED_InitBTDiscoverableSeq(&btDiscoverableSequence);
	sequenceList[seqNum] = &btDiscoverableSequence;
	sequenceInitFuncList[seqNum++] = FUNCTIONAL_RGB_LED_InitBTDiscoverableSeq;

	FUNCTIONAL_RGB_LED_InitBTPairingSeq(&btPairingSequence);
	sequenceList[seqNum] = &btPairingSequence;
	sequenceInitFuncList[seqNum++] = FUNCTIONAL_RGB_LED_InitBTPairingSeq;

	FUNCTIONAL_RGB_LED_InitBTConnectingSeq(&btConnectingSequence);
	sequenceList[seqNum] = &btConnectingSequence;
	sequenceInitFuncList[seqNum++] = FUNCTIONAL_RGB_LED_InitBTConnectingSeq;

	FUNCTIONAL_RGB_LED_InitAUXInShoeSeq(&auxInShoeSequence);
	sequenceList[seqNum] = &auxInShoeSequence;
	sequenceInitFuncList[seqNum++] = FUNCTIONAL_RGB_LED_InitAUXInShoeSeq;

	FUNCTIONAL_RGB_LED_InitAUXInShoe1Seq(&auxInShoe1Sequence);
	sequenceList[seqNum] = &auxInShoe1Sequence;
	sequenceInitFuncList[seqNum++] = FUNCTIONAL_RGB_LED_InitAUXInShoe1Seq;

	FUNCTIONAL_RGB_LED_InitFWUpgradeSeq(&fwUpgradeSequence);
	sequenceList[seqNum] = &fwUpgradeSequence;
	sequenceInitFuncList[seqNum++] = FUNCTIONAL_RGB_LED_InitFWUpgradeSeq;

	FUNCTIONAL_RGB_LED_InitFWUpgradeCompleteSeq(&fwUpgradeCompleteSequence);
	sequenceList[seqNum] = &fwUpgradeCompleteSequence;
	sequenceInitFuncList[seqNum] = FUNCTIONAL_RGB_LED_InitFWUpgradeCompleteSeq;
}

/* ********************************************************************************
** FUNCTION NAME: FUNCTIONAL_RGB_LED_Init()
** DESCRIPTION:
** 				-Initialize the functional rgb led driver
** NOTE:
 *
*********************************************************************************** */
void FUNCTIONAL_RGB_LED_Init(void){
	RGB_LED_Init(FUNC_RGB_LED_NUM);
	return;
}

/* ********************************************************************************
** FUNCTION NAME: FUNCTIONAL_RGB_LED_Start()
** DESCRIPTION:
** 				-Start the functional rgb led driver
** NOTE:
 *
*********************************************************************************** */
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
	ramp_step2.mode = RGB_LIN_MODE; //linear ramp output
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
	hold_step3.duration = 2; //TODO: change to 1 sec later
	hold_step3.next_step_num = 0; //go back to beginning
	hold_step3.last_step = true;
	hold_step3.func_handler = FUNCTIONAL_RGB_LED_Hold;
	hold_step3.complete = false;

	sequence->steps[0] = setpoint_step1;
	sequence->steps[1] = ramp_step2;
	sequence->steps[2] = hold_step3;
	sequence->step_count = 3;
	sequence->complete = false;
	sequence->current_step_num = 0;
	sequence->seq_type = RGBSEQ_ON;
	sequence->loop = false; //don't loop
	sequence->valid = true;

}
void FUNCTIONAL_RGB_LED_InitOffSeq(rgb_led_sequence_t * sequence){
	//step 1: setpoint solid black (off)
	rgb_led_step_t setpoint_step1;
	setpoint_step1.current_step_num = 0;
	setpoint_step1.color_setpoint = BlackColor;
	setpoint_step1.step_type = SETPOINT_STEP;
	setpoint_step1.next_step_num = 1;
	setpoint_step1.last_step = false;
	setpoint_step1.func_handler = FUNCTIONAL_RGB_LED_Setpoint;
	setpoint_step1.complete = false;

	//step 2: exp ramp 500 ms to red
	rgb_led_step_t ramp_step2;
	ramp_step2.time = 0;
	ramp_step2.step_type = RAMP_STEP;
	ramp_step2.mode = RGB_LIN_MODE; //linear ramp output - TODO: but change to exp later
	ramp_step2.current_step_num = 1;
	ramp_step2.color_setpoint = WhiteColor; //TODO: change color to RedColor later
	ramp_step2.duration = 0.5; //500 ms
	ramp_step2.next_step_num = 2;
	ramp_step2.last_step = false;
	ramp_step2.func_handler = FUNCTIONAL_RGB_LED_Ramp;
	ramp_step2.complete = false;

	//step 3: hold 2 sec red
	rgb_led_step_t hold_step3;
	hold_step3.step_type = HOLD_STEP;
	hold_step3.time = 0;
	hold_step3.current_step_num = 2;
	hold_step3.color_setpoint = WhiteColor; //TODO: change to RedColor later
	hold_step3.duration = 2;
	hold_step3.next_step_num = 3; //go back to beginning
	hold_step3.last_step = false;
	hold_step3.func_handler = FUNCTIONAL_RGB_LED_Hold;
	hold_step3.complete = false;

	//step 4: exp ramp 500 ms to black
	rgb_led_step_t ramp_step4;
	ramp_step4.step_type = RAMP_STEP;
	ramp_step4.mode = RGB_LIN_MODE; //linear ramp output
	ramp_step4.time = 0;
	ramp_step4.current_step_num = 3;
	ramp_step4.color_setpoint = BlackColor;
	ramp_step4.duration = 0.5; //500 ms
	ramp_step4.next_step_num = 4;
	ramp_step4.last_step = false;
	ramp_step4.func_handler = FUNCTIONAL_RGB_LED_Ramp;
	ramp_step4.complete = false;

	//step 5: hold 100 ms black
	rgb_led_step_t hold_step5;
	hold_step5.step_type = HOLD_STEP;
	hold_step5.time = 0;
	hold_step5.current_step_num = 4;
	hold_step5.color_setpoint = BlackColor;
	hold_step5.duration = 0.1; //100 ms
	hold_step5.next_step_num = 0; //go back to beginning
	hold_step5.last_step = true;
	hold_step5.func_handler = FUNCTIONAL_RGB_LED_Hold;
	hold_step5.complete = false;


	//fill in steps
	sequence->steps[0] = setpoint_step1;
	sequence->steps[1] = ramp_step2;
	sequence->steps[2] = hold_step3;
	sequence->steps[3] = ramp_step4;
	sequence->steps[4] = hold_step5;
	sequence->step_count = 5;
	sequence->complete = false;
	sequence->current_step_num = 0;
	sequence->seq_type = RGBSEQ_OFF;
	sequence->loop = false; //don't loop
	sequence->valid = true;

}
void FUNCTIONAL_RGB_LED_InitCriticalSeq(rgb_led_sequence_t * sequence){
	sequence->loop = false;
	sequence->enabled = false;
	sequence->valid = false;

}
void FUNCTIONAL_RGB_LED_InitBTConnectedSeq(rgb_led_sequence_t * sequence){
	sequence->loop = false;
	sequence->enabled = false;
	sequence->valid = false;

}
void FUNCTIONAL_RGB_LED_InitBTDiscoverableSeq(rgb_led_sequence_t * sequence){
	sequence->loop = false;
	sequence->enabled = false;
	sequence->valid = false;
}
void FUNCTIONAL_RGB_LED_InitBTPairingSeq(rgb_led_sequence_t * sequence){
	sequence->loop = false;
	sequence->enabled = false;
	sequence->valid = false;
}
void FUNCTIONAL_RGB_LED_InitBTConnectingSeq(rgb_led_sequence_t * sequence){
	sequence->loop = false;
	sequence->enabled = false;
	sequence->valid = false;
}
void FUNCTIONAL_RGB_LED_InitAUXInShoeSeq(rgb_led_sequence_t * sequence){
	sequence->loop = false;
	sequence->enabled = false;
	sequence->valid = false;
}
void FUNCTIONAL_RGB_LED_InitAUXInShoe1Seq(rgb_led_sequence_t * sequence){
	sequence->loop = false;
	sequence->enabled = false;
	sequence->valid = false;
}
void FUNCTIONAL_RGB_LED_InitFWUpgradeSeq(rgb_led_sequence_t * sequence){
	sequence->loop = false;
	sequence->enabled = false;
	sequence->valid = false;
}
void FUNCTIONAL_RGB_LED_InitFWUpgradeCompleteSeq(rgb_led_sequence_t * sequence){
	sequence->loop = false;
	sequence->enabled = false;
	sequence->valid = false;
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
		//get the scale used to increment the led (just run this once)
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
		//increment color and update rgb led driver pwm -- keep doing this until color setpoint reached
		FUNCTIONAL_RGB_LED_GenStepRampOutput(step); //this will generate linear, exponential or log output based on step.mode attribute
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

/* ********************************************************************************
** FUNCTION NAME: FUNCTIONAL_RGB_LED_GenStepRampOutput()
** DESCRIPTION:
** 				- Generate color ramp output based on step mode (exp, log, linear..etc)
** NOTE:
 *
*********************************************************************************** */
void FUNCTIONAL_RGB_LED_GenStepRampOutput(rgb_led_step_t * step){
	switch (step->mode){
	case RGB_LIN_MODE:
		FUNCTIONAL_RGB_LED_SetLinearOutput(step);
		break;
	case RGB_EXP_MODE:
		FUNCTIONAL_RGB_LED_SetExpOutput(step);
		break;
	case RGB_LOG_MODE:
		FUNCTIONAL_RGB_LED_SetLogOutput(step);
		break;
	case RGB_NO_MODE:
		FUNCTIONAL_RGB_LED_SetColor(&step->color);
		break;
	default:
		FUNCTIONAL_RGB_LED_ErrorHandler();

	}
}

/* ********************************************************************************
** FUNCTION NAME: FUNCTIONAL_RGB_LED_SetLinearOutput()
** DESCRIPTION:
** 				- Generate linear color output from step data
** NOTE:
 *
*********************************************************************************** */
void FUNCTIONAL_RGB_LED_SetLinearOutput(rgb_led_step_t * step){
	step->color.red += STEP_TIME_PER_CYCLE * step->scales.red;
	step->color.green += STEP_TIME_PER_CYCLE * step->scales.green;
	step->color.blue += STEP_TIME_PER_CYCLE * step->scales.blue;
	FUNCTIONAL_RGB_LED_SetColor(&step->color);
}

/* ********************************************************************************
** FUNCTION NAME: FUNCTIONAL_RGB_LED_SetExpOutput()
** DESCRIPTION:
** 				- Generate exponential color output from step data
** NOTE:
 *
*********************************************************************************** */
void FUNCTIONAL_RGB_LED_SetExpOutput(rgb_led_step_t * step){
	//TODO: generate exponential output
	FUNCTIONAL_RGB_LED_SetColor(&step->color);
}

/* ********************************************************************************
** FUNCTION NAME: FUNCTIONAL_RGB_LED_SetLogOutput()
** DESCRIPTION:
** 				- Generate logarithmic color output from step data
** NOTE:
 *
*********************************************************************************** */
void FUNCTIONAL_RGB_LED_SetLogOutput(rgb_led_step_t * step){
	//TODO: generate logarithmic output
	FUNCTIONAL_RGB_LED_SetColor(&step->color);
}


/* ********************************************************************************
** FUNCTION NAME: FUNCTIONAL_RGB_LED_InitInterruptTimer()
** DESCRIPTION:
** 				- Setup the timer based interrupt that will call the FUNCTIONAL_RGB_LED_SequenceHandler function to update
** 				  the rgb led colors based on the programmed rgb led functionality (on, off, bluetooth connecting...etc)
** NOTE:
 *
*********************************************************************************** */
void FUNCTIONAL_RGB_LED_InitInterruptTimer(void){
	RGBInterruptTimHandle.Instance = FUNC_RGB_INT_TIM_REG;
	uint32_t timer_freq = HAL_RCC_GetSysClockFreq() / RGB_PRESCALER_DFLT;
	int32_t timer_period = timer_freq / RGB_PWMFREQ_DFLT; //clock will generate interrupt every 20 millisec
	RGBInterruptTimHandle.Init.Period = timer_period;
	RGBInterruptTimHandle.Init.Prescaler = RGB_PRESCALER_DFLT;
	RGBInterruptTimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	RGBInterruptTimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	RGBInterruptTimHandle.Init.RepetitionCounter = 0;

	HAL_TIM_Base_Init(&RGBInterruptTimHandle); //this will also call HAL_TIM_Base_MspInit to init clock and interrupts

	//activate TIM / start with interrupt by calling FUNCTIONAL_RGB_LED_StartInterruptTimer
}

/* ********************************************************************************
** FUNCTION NAME: FUNCTIONAL_RGB_LED_InitInterruptTimer()
** DESCRIPTION:
** 				- Setup the timer based interrupt that will call the FUNCTIONAL_RGB_LED_SequenceHandler function to update
** 				  the rgb led colors based on the programmed rgb led functionality (on, off, bluetooth connecting...etc)
** NOTE:
 *
*********************************************************************************** */
void FUNCTIONAL_RGB_LED_DeInitInterruptTimer(void){

	HAL_TIM_Base_DeInit(&RGBInterruptTimHandle); //this will also call HAL_TIM_Base_MspDeInit to de init clock and interrupts

}

/* ********************************************************************************
** FUNCTION NAME: FUNCTIONAL_RGB_LED_StartInterruptTimer()
** DESCRIPTION:
** 				- Start the timer based interrupt that will call the FUNCTIONAL_RGB_LED_SequenceHandler function to update
** 				  the rgb led colors based on the programmed rgb led functionality (on, off, bluetooth connecting...etc)
** NOTE:
 *
*********************************************************************************** */
void FUNCTIONAL_RGB_LED_StartInterruptTimer(void){
	//make sure you already called the FUNCTIONAL_RGB_LED_InitInterruptTimer function

	//activate TIM / start with interrupt
	HAL_TIM_Base_Start_IT(&RGBInterruptTimHandle);
}

/* ********************************************************************************
** FUNCTION NAME: FUNCTIONAL_RGB_LED_StopInterruptTimer()
** DESCRIPTION:
** 				- Stop the timer based interrupt that calls the FUNCTIONAL_RGB_LED_SequenceHandler function
** NOTE:
 *
*********************************************************************************** */
void FUNCTIONAL_RGB_LED_StopInterruptTimer(void){
	//stop the TIMx interrupt
	HAL_TIM_Base_Stop_IT(&RGBInterruptTimHandle);
}

/* ********************************************************************************
** FUNCTION NAME: FUNCTIONAL_RGB_SequenceHandler()
** DESCRIPTION:
** 				- Runs the sequence step and increments rgb led colors according to step equation
** NOTE:		- You can use the timer interrupt to call this or just call this in a loop but then you need to update the STEP_TIME_PER_CYCLE
 *
*********************************************************************************** */
void FUNCTIONAL_RGB_LED_SequenceHandler(void){
	if(rgbHandle.next_sequence != NULL && rgbHandle.sequence != NULL && !rgbHandle.sequence->enabled){

		if(rgbHandle.next_sequence->seq_type < RGB_SEQ_COUNT){
			rgbHandle.sequence = rgbHandle.next_sequence;
			rgbHandle.next_sequence = NULL;
			sequenceInitFuncList[rgbHandle.sequence->seq_type](rgbHandle.sequence);
		}
	}
	rgb_led_sequence_t * sequence = rgbHandle.sequence;

	if(sequence!= NULL && sequence->enabled && sequence->valid){
		rgb_led_step_t * step;
		 //this just calls the current function pointed to by the step and passes in the step parameters to the function
		 //for ex, the function might be a hold function that just keeps the color at a certain setpoint until the duration is complete
		 //when the duration completes the step.complete attribute is set to true and then the sequence jumps to the next step / function
		 //when the sequence completes the final step is starts all over again from the first step
		  int stepnum = sequence->current_step_num;
		  step = &(sequence->steps[stepnum]);
		  step->func_handler(step);
		  if(step->complete){
			  sequence->current_step_num = step->next_step_num;
			  step->complete = false;
			  if(step->last_step && !sequence->loop){
				  sequence->enabled = false; //stop sequence if the last step finished and the sequence is not set to repeat
			  }
		  }
	  }
	return;
}

void TIM4_IRQHandler(void){
	HAL_TIM_IRQHandler(&RGBInterruptTimHandle);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	//run rgb led sequence routine
	FUNCTIONAL_RGB_LED_SequenceHandler();
}

//called by Timer Initialization
void  HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim){
	RGB_LED_EnTimClk(htim->Instance);
	HAL_NVIC_SetPriority((IRQn_Type)(FUNC_RGB_INT_TIM_IRQ), 0x0F, 0x00);
	HAL_NVIC_EnableIRQ((IRQn_Type)(FUNC_RGB_INT_TIM_IRQ));
}

//called by Timer DeInitialization
void  HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim){
	RGB_LED_DisTimClk(htim->Instance);
	HAL_NVIC_DisableIRQ((IRQn_Type)(FUNC_RGB_INT_TIM_IRQ));
}


