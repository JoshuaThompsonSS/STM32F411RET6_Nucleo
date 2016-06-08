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
			//rgbHandle.next_sequence->enabled = true;
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
** FUNCTION NAME: FUNCTIONAL_RGB_LED_InitCmdLine()
** DESCRIPTION:
** 				-Initialize cmd line testing variables and other stuff
** NOTE:
 *
*********************************************************************************** */

void FUNCTIONAL_RGB_LED_InitCmdLine(void){
	CrtclSeqUpDur = 0.10;
	CrtclSeqDwnDur = 0.10;
	CrtclSeqHldDur = 0.500;
	CrtclSeqMode = RGB_LIN_MODE;
}

/* ********************************************************************************
** FUNCTION NAME: FUNCTIONAL_RGB_LED_StartService()
** DESCRIPTION:
** 				-Initialize driver, interrupt routine and structures and then start the rgb led sequence interrupt driven routine
** NOTE:
 *
*********************************************************************************** */
void FUNCTIONAL_RGB_LED_StartService(void){
	//Cmd line testing
	FUNCTIONAL_RGB_LED_InitCmdLine();
	//init rgb led driver handle (rgbHandle)
	FUNCTIONAL_RGB_LED_InitHandle();
	//init all the common functional rgb led sequences
	FUNCTIONAL_RGB_LED_InitSequences();
	//init driver
	rgbHandle.enabled = true;
	FUNCTIONAL_RGB_LED_Init();
	//start driver
	FUNCTIONAL_RGB_LED_Start();

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
	rgbHandle.enabled = false;
	FUNCTIONAL_RGB_LED_Stop();

	//init driver
	FUNCTIONAL_RGB_LED_DeInit();

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

	FUNCTIONAL_RGB_LED_InitOnSeq(&onSequence);
	sequenceList[ON_SEQ_TYPE] = &onSequence;
	sequenceInitFuncList[ON_SEQ_TYPE] = FUNCTIONAL_RGB_LED_InitOnSeq;

	FUNCTIONAL_RGB_LED_InitOffSeq(&offSequence);
	sequenceList[OFF_SEQ_TYPE] = &offSequence;
	sequenceInitFuncList[OFF_SEQ_TYPE] = FUNCTIONAL_RGB_LED_InitOffSeq;

	FUNCTIONAL_RGB_LED_InitCriticalSeq(&criticalBattSequence);
	sequenceList[CRTCL_SEQ_TYPE] = &criticalBattSequence;
	sequenceInitFuncList[CRTCL_SEQ_TYPE] = FUNCTIONAL_RGB_LED_InitCriticalSeq;

	FUNCTIONAL_RGB_LED_InitChargingSeq(&chargingSequence);
	sequenceList[CHRNG_SEQ_TYPE] = &chargingSequence;
	sequenceInitFuncList[CHRNG_SEQ_TYPE] = FUNCTIONAL_RGB_LED_InitChargingSeq;

	FUNCTIONAL_RGB_LED_InitChargedSeq(&chargedSequence);
	sequenceList[CHRGD_SEQ_TYPE] = &chargedSequence;
	sequenceInitFuncList[CHRGD_SEQ_TYPE] = FUNCTIONAL_RGB_LED_InitChargedSeq;

	FUNCTIONAL_RGB_LED_InitResetSeq(&resetSequence);
	sequenceList[RST_SEQ_TYPE] = &resetSequence;
	sequenceInitFuncList[RST_SEQ_TYPE] = FUNCTIONAL_RGB_LED_InitResetSeq;

	FUNCTIONAL_RGB_LED_InitBTConnectedSeq(&btConnectedSequence);
	sequenceList[BTCNTD_SEQ_TYPE] = &btConnectedSequence;
	sequenceInitFuncList[BTCNTD_SEQ_TYPE] = FUNCTIONAL_RGB_LED_InitBTConnectedSeq;

	FUNCTIONAL_RGB_LED_InitBTDiscoverableSeq(&btDiscoverableSequence);
	sequenceList[BTDISC_SEQ_TYPE] = &btDiscoverableSequence;
	sequenceInitFuncList[BTDISC_SEQ_TYPE] = FUNCTIONAL_RGB_LED_InitBTDiscoverableSeq;

	FUNCTIONAL_RGB_LED_InitBTPairingSeq(&btPairingSequence);
	sequenceList[BTPR_SEQ_TYPE] = &btPairingSequence;
	sequenceInitFuncList[BTPR_SEQ_TYPE] = FUNCTIONAL_RGB_LED_InitBTPairingSeq;

	FUNCTIONAL_RGB_LED_InitBTConnectingSeq(&btConnectingSequence);
	sequenceList[BTCNTG_SEQ_TYPE] = &btConnectingSequence;
	sequenceInitFuncList[BTCNTG_SEQ_TYPE] = FUNCTIONAL_RGB_LED_InitBTConnectingSeq;

	FUNCTIONAL_RGB_LED_InitAUXInShoeSeq(&auxInShoeSequence);
	sequenceList[AXSH_SEQ_TYPE] = &auxInShoeSequence;
	sequenceInitFuncList[AXSH_SEQ_TYPE] = FUNCTIONAL_RGB_LED_InitAUXInShoeSeq;

	FUNCTIONAL_RGB_LED_InitAUXInShoe1No2Seq(&auxInShoe1No2Sequence);
	sequenceList[AXSH1N2_SEQ_TYPE] = &auxInShoe1No2Sequence;
	sequenceInitFuncList[AXSH1N2_SEQ_TYPE] = FUNCTIONAL_RGB_LED_InitAUXInShoe1No2Seq;

	FUNCTIONAL_RGB_LED_InitFWUpgradeShoe1Seq(&fwUpgradeShoe1Sequence);
	sequenceList[FWUPSH1_SEQ_TYPE] = &fwUpgradeShoe1Sequence;
	sequenceInitFuncList[FWUPSH1_SEQ_TYPE] = FUNCTIONAL_RGB_LED_InitFWUpgradeShoe1Seq;

	FUNCTIONAL_RGB_LED_InitFWUpgradeShoe2Seq(&fwUpgradeShoe2Sequence);
	sequenceList[FWUPSH2_SEQ_TYPE] = &fwUpgradeShoe2Sequence;
	sequenceInitFuncList[FWUPSH2_SEQ_TYPE] = FUNCTIONAL_RGB_LED_InitFWUpgradeShoe2Seq;

	FUNCTIONAL_RGB_LED_InitFWUpgradeCompleteSeq(&fwUpgradeCompleteSequence);
	sequenceList[FWUPCPLT_SEQ_TYPE] = &fwUpgradeCompleteSequence;
	sequenceInitFuncList[FWUPCPLT_SEQ_TYPE] = FUNCTIONAL_RGB_LED_InitFWUpgradeCompleteSeq;
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


//Init Sequences
void FUNCTIONAL_RGB_LED_InitOnSeq(rgb_led_sequence_t * sequence){
	//step 1: Setpoint Step - set solid color black (off)
	FUNCTIONAL_RGB_LED_InitSetpointStep(&sequence->steps[0], false, 0, 1, BlackColor);

	//step 2: Ramp Step - increase color to white at a duration of 1 sec
	FUNCTIONAL_RGB_LED_InitRampStep(&sequence->steps[1], false, 1, 2, WhiteColor, 1.00, RGB_LIN_MODE);

	//step 3: Hold Step - set color to white and hold for duration of 1 sec
	//			- make this step the last step (so sequence repeats)
	FUNCTIONAL_RGB_LED_InitHoldStep(&sequence->steps[2], true, 2, 0, WhiteColor, 2.00);

	sequence->step_count = 3;
	sequence->complete = false;
	sequence->current_step_num = 0;
	sequence->seq_type = RGBSEQ_ON;
	sequence->loop = false; //don't loop
	sequence->valid = true;

}
void FUNCTIONAL_RGB_LED_InitOffSeq(rgb_led_sequence_t * sequence){
	//step 1: setpoint solid black (off)
	FUNCTIONAL_RGB_LED_InitSetpointStep(&sequence->steps[0], false, 0, 1, BlackColor);


	//step 2: exp ramp 500 ms to red (linear ramp for now, and use white color for now)
	FUNCTIONAL_RGB_LED_InitRampStep(&sequence->steps[1], false, 1, 2, WhiteColor, 0.500, RGB_LIN_MODE);


	//step 3: hold 2 sec red
	FUNCTIONAL_RGB_LED_InitHoldStep(&sequence->steps[2], false, 2, 3, WhiteColor, 2.00);


	//step 4: exp ramp 500 ms to black (linear ramp for now)
	FUNCTIONAL_RGB_LED_InitRampStep(&sequence->steps[3], false, 3, 4, BlackColor, 0.50, RGB_LIN_MODE);


	//step 5: hold 100 ms black then stop (last step = true)
	FUNCTIONAL_RGB_LED_InitHoldStep(&sequence->steps[4], true, 4, 0, BlackColor, 0.100);


	//fill in steps
	sequence->step_count = 5;
	sequence->complete = false;
	sequence->current_step_num = 0;
	sequence->seq_type = RGBSEQ_OFF;
	sequence->loop = false; //don't loop
	sequence->valid = true;

}
void FUNCTIONAL_RGB_LED_InitCriticalSeq(rgb_led_sequence_t * sequence){
	//step 1: setpoint solid black (off)
	FUNCTIONAL_RGB_LED_InitSetpointStep(&sequence->steps[0], false, 0, 1, BlackColor);

	//step 2: exp ramp 250 ms to red (linear ramp for now, and use white color for now)
	//0.250
	FUNCTIONAL_RGB_LED_InitRampStep(&sequence->steps[1], false, 1, 2, WhiteColor, CrtclSeqUpDur, CrtclSeqMode);
	//TODO: change to red

	//step 3: hold 500 millisec red
	FUNCTIONAL_RGB_LED_InitHoldStep(&sequence->steps[2], false, 2, 3, WhiteColor, CrtclSeqHldDur);
	//TODO: change to red

	//step 4: exp ramp 250 ms to black (linear ramp for now)
	//250 ms
	FUNCTIONAL_RGB_LED_InitRampStep(&sequence->steps[3], false, 3, 4, BlackColor, CrtclSeqDwnDur, CrtclSeqMode); //blackcolor


	//step 5: hold 500 ms black then stop
	FUNCTIONAL_RGB_LED_InitHoldStep(&sequence->steps[4], false, 4, 5, BlackColor, CrtclSeqHldDur); //0.500

	//step 6: repeat steps 2 - 5 (1 - 4 using 0 index) - last step = true
	FUNCTIONAL_RGB_LED_InitRepeatStep(&sequence->steps[5], true, 5, 1, 1);


	//fill in steps
	sequence->step_count = 6;
	sequence->complete = false;
	sequence->current_step_num = 0;
	sequence->seq_type = RGBSEQ_CRITICAL;
	sequence->loop = true; //loop
	sequence->valid = true;

}
void FUNCTIONAL_RGB_LED_InitChargingSeq(rgb_led_sequence_t * sequence){
	//step 1: setpoint solid black (off)
	FUNCTIONAL_RGB_LED_InitSetpointStep(&sequence->steps[0], false, 0, 1, BlackColor);


	//step 2: exp ramp 250 ms to white (linear ramp for now)
	FUNCTIONAL_RGB_LED_InitRampStep(&sequence->steps[1], false, 1, 2, WhiteColor, 0.250, RGB_LIN_MODE);


	//step 3: hold 500 sec white
	FUNCTIONAL_RGB_LED_InitHoldStep(&sequence->steps[2], false, 2, 3, WhiteColor, 0.500);


	//step 4: exp ramp 250 ms to black (linear ramp for now)
	FUNCTIONAL_RGB_LED_InitRampStep(&sequence->steps[3], false, 3, 4, BlackColor, 0.250, RGB_LIN_MODE);


	//step 5: hold 500 ms black then stop
	FUNCTIONAL_RGB_LED_InitHoldStep(&sequence->steps[4], false, 4, 5, BlackColor, 0.500);

	//step 6: repeat steps 2 - 5 (1 - 4 using 0 index) - last step = true
	FUNCTIONAL_RGB_LED_InitRepeatStep(&sequence->steps[5], true, 5, 1, 1);


	//fill in steps
	sequence->step_count = 6;
	sequence->complete = false;
	sequence->current_step_num = 0;
	sequence->seq_type = RGBSEQ_CHARGING;
	sequence->loop = true; //loop
	sequence->valid = true;

}
void FUNCTIONAL_RGB_LED_InitChargedSeq(rgb_led_sequence_t * sequence){
	//step 1: Setpoint Step - set solid color black (off)
	FUNCTIONAL_RGB_LED_InitSetpointStep(&sequence->steps[0], false, 0, 1, BlackColor);

	//step 2: Ramp Step - increase color to white at a duration of 500 ms
	FUNCTIONAL_RGB_LED_InitRampStep(&sequence->steps[1], false, 1, 2, WhiteColor, 0.500, RGB_LIN_MODE);

	//step 3: Setpoint Step - set color to white
	FUNCTIONAL_RGB_LED_InitSetpointStep(&sequence->steps[2], true, 2, 0, WhiteColor);

	sequence->step_count = 3;
	sequence->complete = false;
	sequence->current_step_num = 0;
	sequence->seq_type = RGBSEQ_CHARGED;
	sequence->loop = false; //don't loop
	sequence->valid = true;

}
void FUNCTIONAL_RGB_LED_InitResetSeq(rgb_led_sequence_t * sequence){
	//step 1: setpoint solid black (off)
	FUNCTIONAL_RGB_LED_InitSetpointStep(&sequence->steps[0], false, 0, 1, BlackColor);


	//step 2: exp ramp 250 ms to red (linear ramp for now, white for now)
	FUNCTIONAL_RGB_LED_InitRampStep(&sequence->steps[1], false, 1, 2, WhiteColor, 0.250, RGB_LIN_MODE);
	//TODO: change to red

	//step 3: hold 500 sec red (white for now)
	FUNCTIONAL_RGB_LED_InitHoldStep(&sequence->steps[2], false, 2, 3, WhiteColor, 0.500);


	//step 4: exp ramp 250 ms to black (linear ramp for now)
	FUNCTIONAL_RGB_LED_InitRampStep(&sequence->steps[3], false, 3, 4, BlackColor, 0.250, RGB_LIN_MODE);


	//step 5: hold 500 ms black then stop
	FUNCTIONAL_RGB_LED_InitHoldStep(&sequence->steps[4], false, 4, 5, BlackColor, 0.500);

	//step 6: exp ramp 250 ms to red (linear ramp for now, white for now)
	FUNCTIONAL_RGB_LED_InitRampStep(&sequence->steps[5], false, 5, 6, WhiteColor, 0.250, RGB_LIN_MODE);

	//step 7: hold 4 sec sold red (white for now)
	FUNCTIONAL_RGB_LED_InitHoldStep(&sequence->steps[6], false, 6, 7, WhiteColor, 4.0);
	//TODO: change to red

	//step 8: exp ramp 250 ms to black
	FUNCTIONAL_RGB_LED_InitRampStep(&sequence->steps[7], false, 7, 8, BlackColor, 0.250, RGB_LIN_MODE);

	//step 9: hold 100 ms solid black
	FUNCTIONAL_RGB_LED_InitHoldStep(&sequence->steps[8], true, 8, 0, BlackColor, 0.100);


	//fill in steps
	sequence->step_count = 9;
	sequence->complete = false;
	sequence->current_step_num = 0;
	sequence->seq_type = RGBSEQ_RESET_SHOE;
	sequence->loop = false; //no loop
	sequence->valid = true;

}
void FUNCTIONAL_RGB_LED_InitBTConnectedSeq(rgb_led_sequence_t * sequence){
	//step 1: Setpoint Step - set solid color black (off)
	FUNCTIONAL_RGB_LED_InitSetpointStep(&sequence->steps[0], false, 0, 1, BlackColor);

	//step 2: Ramp Step - increase color to white at a duration of 1 sec
	FUNCTIONAL_RGB_LED_InitRampStep(&sequence->steps[1], false, 1, 2, WhiteColor, 0.500, RGB_LIN_MODE);

	//step 3: Hold Step - set color to white and hold for duration of 1 sec
	//			- make this step the last step (so sequence repeats)
	FUNCTIONAL_RGB_LED_InitSetpointStep(&sequence->steps[2], true, 2, 0, WhiteColor);

	sequence->step_count = 3;
	sequence->complete = false;
	sequence->current_step_num = 0;
	sequence->seq_type = RGBSEQ_BT_CONNECTED;
	sequence->loop = false; //don't loop
	sequence->valid = true;

}
void FUNCTIONAL_RGB_LED_InitBTDiscoverableSeq(rgb_led_sequence_t * sequence){
	//TODO: change white to blue

	//step 1: setpoint solid black (off)
	FUNCTIONAL_RGB_LED_InitSetpointStep(&sequence->steps[0], false, 0, 1, BlackColor);


	//step 2: exp ramp 250 ms to blue (linear ramp for now, and use white color for now)
	FUNCTIONAL_RGB_LED_InitRampStep(&sequence->steps[1], false, 1, 2, WhiteColor, 0.250, RGB_LIN_MODE);


	//step 3: hold 500 ms sec blue (white for now)
	FUNCTIONAL_RGB_LED_InitHoldStep(&sequence->steps[2], false, 2, 3, WhiteColor, 0.500);


	//step 4: exp ramp 250 ms to black (linear ramp for now)
	FUNCTIONAL_RGB_LED_InitRampStep(&sequence->steps[3], false, 3, 4, BlackColor, 0.250, RGB_LIN_MODE);


	//step 5: hold 500 ms black then stop (last step = true)
	FUNCTIONAL_RGB_LED_InitHoldStep(&sequence->steps[4], true, 4, 0, BlackColor, 0.500);


	//fill in steps
	sequence->step_count = 5;
	sequence->complete = false;
	sequence->current_step_num = 0;
	sequence->seq_type = RGBSEQ_BT_DISCOVERABLE;
	sequence->loop = true; //loop until connected
	sequence->valid = true;
}
void FUNCTIONAL_RGB_LED_InitBTPairingSeq(rgb_led_sequence_t * sequence){
	//TODO: change white to blue

	//step 1: setpoint solid black (off)
	FUNCTIONAL_RGB_LED_InitSetpointStep(&sequence->steps[0], false, 0, 1, BlackColor);


	//step 2: exp ramp 250 ms to blue (linear ramp for now, and use white color for now)
	FUNCTIONAL_RGB_LED_InitRampStep(&sequence->steps[1], false, 1, 2, WhiteColor, 0.250, RGB_LIN_MODE);


	//step 3: hold 500 ms sec blue (white for now)
	FUNCTIONAL_RGB_LED_InitHoldStep(&sequence->steps[2], false, 2, 3, WhiteColor, 0.500);


	//step 4: exp ramp 250 ms to black (linear ramp for now)
	FUNCTIONAL_RGB_LED_InitRampStep(&sequence->steps[3], false, 3, 4, BlackColor, 0.250, RGB_LIN_MODE);


	//step 5: hold 500 ms black then stop (last step = true)
	FUNCTIONAL_RGB_LED_InitHoldStep(&sequence->steps[4], true, 4, 0, BlackColor, 0.500);


	//fill in steps
	sequence->step_count = 5;
	sequence->complete = false;
	sequence->current_step_num = 0;
	sequence->seq_type = RGBSEQ_BT_PAIRING;
	sequence->loop = true; //loop until connected
	sequence->valid = true;
}
void FUNCTIONAL_RGB_LED_InitBTConnectingSeq(rgb_led_sequence_t * sequence){
	//TODO: change white to blue

	//step 1: setpoint solid black (off)
	FUNCTIONAL_RGB_LED_InitSetpointStep(&sequence->steps[0], false, 0, 1, BlackColor);


	//step 2: exp ramp 250 ms to blue (linear ramp for now, and use white color for now)
	FUNCTIONAL_RGB_LED_InitRampStep(&sequence->steps[1], false, 1, 2, WhiteColor, 0.250, RGB_LIN_MODE);


	//step 3: hold 500 ms sec blue (white for now)
	FUNCTIONAL_RGB_LED_InitHoldStep(&sequence->steps[2], false, 2, 3, WhiteColor, 0.500);


	//step 4: exp ramp 250 ms to black (linear ramp for now)
	FUNCTIONAL_RGB_LED_InitRampStep(&sequence->steps[3], false, 3, 4, BlackColor, 0.250, RGB_LIN_MODE);


	//step 5: hold 500 ms black then stop (last step = true)
	FUNCTIONAL_RGB_LED_InitHoldStep(&sequence->steps[4], true, 4, 0, BlackColor, 0.500);


	//fill in steps
	sequence->step_count = 5;
	sequence->complete = false;
	sequence->current_step_num = 0;
	sequence->seq_type = RGBSEQ_BT_CONNECTING;
	sequence->loop = true; //loop until connected
	sequence->valid = true;
}
void FUNCTIONAL_RGB_LED_InitAUXInShoeSeq(rgb_led_sequence_t * sequence){
	//step 1: Setpoint Step - set solid color black (off)
	FUNCTIONAL_RGB_LED_InitSetpointStep(&sequence->steps[0], false, 0, 1, BlackColor);

	//step 2: Ramp Step - increase color to white at a duration of 500 ms
	FUNCTIONAL_RGB_LED_InitRampStep(&sequence->steps[1], false, 1, 2, WhiteColor, 0.500, RGB_LIN_MODE);

	//step 3: Hold Step - set color to white
	FUNCTIONAL_RGB_LED_InitSetpointStep(&sequence->steps[2], true, 2, 0, WhiteColor);

	sequence->step_count = 3;
	sequence->complete = false;
	sequence->current_step_num = 0;
	sequence->seq_type = RGBSEQ_AUX_IN_SHOE;
	sequence->loop = false; //don't loop
	sequence->valid = true;
}
void FUNCTIONAL_RGB_LED_InitAUXInShoe1No2Seq(rgb_led_sequence_t * sequence){

	//step 1: setpoint solid black (off)
	FUNCTIONAL_RGB_LED_InitSetpointStep(&sequence->steps[0], false, 0, 1, BlackColor);


	//step 2: exp ramp 250 ms to blue (linear ramp for now, and use white color for now)
	FUNCTIONAL_RGB_LED_InitRampStep(&sequence->steps[1], false, 1, 2, WhiteColor, 0.250, RGB_LIN_MODE);


	//step 3: hold 500 ms sec blue (white for now)
	FUNCTIONAL_RGB_LED_InitHoldStep(&sequence->steps[2], false, 2, 3, WhiteColor, 0.500);


	//step 4: exp ramp 250 ms to black (linear ramp for now)
	FUNCTIONAL_RGB_LED_InitRampStep(&sequence->steps[3], false, 3, 4, BlackColor, 0.250, RGB_LIN_MODE);


	//step 5: hold 500 ms black then stop (last step = true)
	FUNCTIONAL_RGB_LED_InitHoldStep(&sequence->steps[4], true, 4, 0, BlackColor, 0.500);


	//fill in steps
	sequence->step_count = 5;
	sequence->complete = false;
	sequence->current_step_num = 0;
	sequence->seq_type = RGBSEQ_AUX_IN_SHOE1NO2;
	sequence->loop = true; //loop until both shoes connected
	sequence->valid = true;
}


void FUNCTIONAL_RGB_LED_InitFWUpgradeShoe1Seq(rgb_led_sequence_t * sequence){
	//step 1: setpoint solid black (off)
	FUNCTIONAL_RGB_LED_InitSetpointStep(&sequence->steps[0], false, 0, 1, BlackColor);


	//step 2: exp ramp 250 ms to white (linear ramp for now, white for now)
	FUNCTIONAL_RGB_LED_InitRampStep(&sequence->steps[1], false, 1, 2, WhiteColor, 0.250, RGB_LIN_MODE);

	//step 3: hold 1 sec red (white for now)
	FUNCTIONAL_RGB_LED_InitHoldStep(&sequence->steps[2], false, 2, 3, WhiteColor, 1.0);


	//step 4: exp ramp 250 ms to black (linear ramp for now)
	FUNCTIONAL_RGB_LED_InitRampStep(&sequence->steps[3], false, 3, 4, BlackColor, 0.250, RGB_LIN_MODE);


	//step 5: hold 1 sec black
	FUNCTIONAL_RGB_LED_InitHoldStep(&sequence->steps[4], false, 4, 5, BlackColor, 1.0);

	//step 6: exp ramp 250 ms to white (linear ramp for now)
	FUNCTIONAL_RGB_LED_InitRampStep(&sequence->steps[5], false, 5, 6, WhiteColor, 0.250, RGB_LIN_MODE);

	//step 7: hold 250 ms sold white
	FUNCTIONAL_RGB_LED_InitHoldStep(&sequence->steps[6], false, 6, 7, WhiteColor, 0.250);
	//TODO: change to red

	//step 8: exp ramp 250 ms to black
	FUNCTIONAL_RGB_LED_InitRampStep(&sequence->steps[7], false, 7, 8, BlackColor, 0.250, RGB_LIN_MODE);

	//step 9: hold 250 ms solid black
	FUNCTIONAL_RGB_LED_InitHoldStep(&sequence->steps[8], true, 8, 0, BlackColor, 0.250);


	//fill in steps
	sequence->step_count = 9;
	sequence->complete = false;
	sequence->current_step_num = 0;
	sequence->seq_type = RGBSEQ_FW_UPGRADE_SHOE1;
	sequence->loop = true; //repeat until fw upload complete
	sequence->valid = true;
}

void FUNCTIONAL_RGB_LED_InitFWUpgradeShoe2Seq(rgb_led_sequence_t * sequence){
	//fast blink
	//step 1: setpoint solid black (off)
	FUNCTIONAL_RGB_LED_InitSetpointStep(&sequence->steps[0], false, 0, 1, BlackColor);


	//step 2: exp ramp 250 ms to white (linear ramp for now, white for now)
	FUNCTIONAL_RGB_LED_InitRampStep(&sequence->steps[1], false, 1, 2, WhiteColor, 0.250, RGB_LIN_MODE);

	//step 3: hold 250 ms red (white for now)
	FUNCTIONAL_RGB_LED_InitHoldStep(&sequence->steps[2], false, 2, 3, WhiteColor, 0.250);


	//step 4: exp ramp 250 ms to black (linear ramp for now)
	FUNCTIONAL_RGB_LED_InitRampStep(&sequence->steps[3], false, 3, 4, BlackColor, 0.250, RGB_LIN_MODE);


	//step 5: hold 1 sec black
	FUNCTIONAL_RGB_LED_InitHoldStep(&sequence->steps[4], false, 4, 5, BlackColor, 1.0);

	//slow blink
	//step 6: exp ramp 250 ms to white (linear ramp for now)
	FUNCTIONAL_RGB_LED_InitRampStep(&sequence->steps[5], false, 5, 6, WhiteColor, 0.250, RGB_LIN_MODE);

	//step 7: hold 1 sec sold white
	FUNCTIONAL_RGB_LED_InitHoldStep(&sequence->steps[6], false, 6, 7, WhiteColor, 1.0);
	//TODO: change to red

	//step 8: exp ramp 250 ms to black
	FUNCTIONAL_RGB_LED_InitRampStep(&sequence->steps[7], false, 7, 8, BlackColor, 0.250, RGB_LIN_MODE);

	//step 9: hold 1 sec solid black
	FUNCTIONAL_RGB_LED_InitHoldStep(&sequence->steps[8], true, 8, 0, BlackColor, 1.0);


	//fill in steps
	sequence->step_count = 9;
	sequence->complete = false;
	sequence->current_step_num = 0;
	sequence->seq_type = RGBSEQ_FW_UPGRADE_SHOE2;
	sequence->loop = true; //repeat until fw upload complete
	sequence->valid = true;
}



void FUNCTIONAL_RGB_LED_InitFWUpgradeCompleteSeq(rgb_led_sequence_t * sequence){
	//step 1: setpoint solid black (off)
	FUNCTIONAL_RGB_LED_InitSetpointStep(&sequence->steps[0], false, 0, 1, BlackColor);


	//step 2: exp ramp 250 ms to red (linear ramp for now, white for now)
	FUNCTIONAL_RGB_LED_InitRampStep(&sequence->steps[1], false, 1, 2, WhiteColor, 0.250, RGB_LIN_MODE);
	//TODO: change to red

	//step 3: hold 500 sec red (white for now)
	FUNCTIONAL_RGB_LED_InitHoldStep(&sequence->steps[2], false, 2, 3, WhiteColor, 0.500);


	//step 4: exp ramp 250 ms to black (linear ramp for now)
	FUNCTIONAL_RGB_LED_InitRampStep(&sequence->steps[3], false, 3, 4, BlackColor, 0.250, RGB_LIN_MODE);


	//step 5: hold 500 ms black then stop
	FUNCTIONAL_RGB_LED_InitHoldStep(&sequence->steps[4], false, 4, 5, BlackColor, 0.500);

	//step 6: exp ramp 250 ms to red (linear ramp for now, white for now)
	FUNCTIONAL_RGB_LED_InitRampStep(&sequence->steps[5], false, 5, 6, WhiteColor, 0.250, RGB_LIN_MODE);

	//step 7: hold 4 sec sold red (white for now)
	FUNCTIONAL_RGB_LED_InitHoldStep(&sequence->steps[6], false, 6, 7, WhiteColor, 4.0);
	//TODO: change to red

	//step 8: exp ramp 250 ms to black
	FUNCTIONAL_RGB_LED_InitRampStep(&sequence->steps[7], false, 7, 8, BlackColor, 0.250, RGB_LIN_MODE);

	//step 9: hold 100 ms solid black
	FUNCTIONAL_RGB_LED_InitHoldStep(&sequence->steps[8], true, 8, 0, BlackColor, 0.100);


	//fill in steps
	sequence->step_count = 9;
	sequence->complete = false;
	sequence->current_step_num = 0;
	sequence->seq_type = RGBSEQ_FW_UPGRADE_COMPLETE;
	sequence->loop = false; //no loop
	sequence->valid = true;
}


//Init / Create Step
/* ********************************************************************************
** FUNCTION NAME: FUNCTIONAL_RGB_LED_InitRampStep()
** DESCRIPTION:
** 				- Initializes step structure to run a ramp rgb led step
** NOTE:
 *
*********************************************************************************** */
void FUNCTIONAL_RGB_LED_InitRampStep(rgb_led_step_t * step, bool last, int crnt_num, int nxt_num, rgb_color_t color, float duration, rgb_modes_t mode){
	step->current_step_num = crnt_num;
	step->next_step_num = nxt_num;
	step->mode = mode; //linear, exp, log...etc
	step->color_setpoint = color;
	step->step_type = RAMP_STEP;
	step->time = 0;
	step->duration = duration;
	step->last_step = last;
	step->func_handler = FUNCTIONAL_RGB_LED_Ramp; //function that will update the actual rgb led pwm signals based on mode
	step->complete = false;

}

/* ********************************************************************************
** FUNCTION NAME: FUNCTIONAL_RGB_LED_InitHoldStep()
** DESCRIPTION:
** 				- - Initializes step structure to run a hold rgb led step
** NOTE:
 *
*********************************************************************************** */
void FUNCTIONAL_RGB_LED_InitHoldStep(rgb_led_step_t * step, bool last, int crnt_num, int nxt_num, rgb_color_t color, float duration){
	step->current_step_num = crnt_num;
	step->next_step_num = nxt_num;
	step->mode = RGB_NO_MODE;
	step->color_setpoint = color;
	step->step_type = HOLD_STEP;
	step->time = 0;
	step->duration = duration;
	step->last_step = last;
	step->func_handler = FUNCTIONAL_RGB_LED_Hold; //function that will update the actual rgb led pwm signals based on mode
	step->complete = false;

}

/* ********************************************************************************
** FUNCTION NAME: FUNCTIONAL_RGB_LED_InitSetpointStep()
** DESCRIPTION:
** 				- Initializes step structure to run a setpoint rgb led step
** NOTE:
 *
*********************************************************************************** */
void FUNCTIONAL_RGB_LED_InitSetpointStep(rgb_led_step_t * step, bool last, int crnt_num, int nxt_num, rgb_color_t color){
	step->current_step_num = crnt_num;
	step->next_step_num = nxt_num;
	step->mode = RGB_NO_MODE;
	step->color_setpoint = color;
	step->step_type = SETPOINT_STEP;
	step->last_step = last;
	step->func_handler = FUNCTIONAL_RGB_LED_Setpoint; //function that will update the actual rgb led pwm signals based on mode
	step->complete = false;

}

/* ********************************************************************************
** FUNCTION NAME: FUNCTIONAL_RGB_LED_InitRepeatStep()
** DESCRIPTION:
** 				- Initializes step structure to run a setpoint rgb led step
** NOTE:
 *
*********************************************************************************** */
void FUNCTIONAL_RGB_LED_InitRepeatStep(rgb_led_step_t * step, bool last, int crnt_num, int nxt_num, float duration){
	step->current_step_num = crnt_num;
	step->next_step_num = nxt_num;
	step->mode = RGB_NO_MODE;
	step->step_type = REPEAT_STEP;
	step->duration = duration;
	step->last_step = last;
	step->func_handler = FUNCTIONAL_RGB_LED_Repeat; //function that will update the actual rgb led pwm signals based on mode
	step->complete = false;

}

/* ********************************************************************************
** FUNCTION NAME: FUNCTIONAL_RGB_LED_GetColorScales()
** DESCRIPTION:
** 				- calculate slope for either linear, log or exponential ramp equation
** NOTE:
 *
*********************************************************************************** */
void FUNCTIONAL_RGB_LED_GetColorScales(rgb_led_step_t * step){
	switch(step->mode){
	case RGB_LIN_MODE:
		step->scales.red = (step->color_setpoint.red - step->color_offsets.red) / step->duration;
		step->scales.green = (step->color_setpoint.green - step->color_offsets.green) / step->duration;
		step->scales.blue = (step->color_setpoint.green - step->color_offsets.green) / step->duration;
		break;
	case RGB_EXP_MODE:
		step->scales.red = (step->color_setpoint.red - step->color_offsets.red)  / (step->duration*step->duration * step->duration);
		step->scales.green = (step->color_setpoint.green - step->color_offsets.green)  / (step->duration*step->duration *step->duration);
		step->scales.blue = (step->color_setpoint.blue - step->color_offsets.blue)  / (step->duration*step->duration *step->duration);
		break;
	case RGB_LOG_MODE:
		//nothing dev yet
		break;
	default:
		break;
	}
	return;
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

		step->color_offsets.red = step->color.red;
		step->color_offsets.green = step->color.green;
		step->color_offsets.blue = step->color.blue;
		//get slopes for exp (just x^2 for now) or linear equations (y = mx + b)
		FUNCTIONAL_RGB_LED_GetColorScales(step);

	}
	step->time += STEP_TIME_PER_CYCLE; //time for each step interrupt loop
	int color_diff = dlta_clr.red + dlta_clr.green + dlta_clr.blue;
	if(color_diff < 0)
	{
		color_diff = color_diff * (-1);
	} //abs value


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
	//this step just keeps a count of how many times the sequences is told to go back to the beginning,
	//or whatever step is pointed to by the repeat step next_step_num
	if(step->time >= step->duration){
		step->complete = true; //repeat loops are finished
		step->time = 0; //clear for next use
		return;
	}
	else{
		//increment counter / timer everytime sequence gets to this step
		step->time++;
		//sequence will look for the repeat steps next_step_num and will then go there until it gets here again
	}

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

	float t = step->time;
	//step->color.red += (STEP_TIME_PER_CYCLE * redscale);
	step->color.red = (t * step->scales.red) + step->color_offsets.red;
	step->color.green = (t * step->scales.green) + step->color_offsets.green;
	step->color.blue = (t * step->scales.blue) + step->color_offsets.blue;
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
	//output = scale(x^3) + offset
	float t = step->time;
	step->color.red = step->scales.red*(t*t*t) + step->color_offsets.red;
	step->color.green = step->scales.green*(t*t*t) + step->color_offsets.green;
	step->color.blue = step->scales.blue*(t*t*t) + step->color_offsets.blue;

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
	//if time/duration >= 50% then use line equation 2
	//else scale with normal linear output
	//output = c*log(scale*(time + 1)) + offset
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
	if(!rgbHandle.enabled){return;}
	if(rgbHandle.next_sequence != NULL && rgbHandle.sequence != NULL && !rgbHandle.sequence->enabled){

		if(rgbHandle.next_sequence->seq_type < RGB_SEQ_COUNT){
			rgbHandle.sequence = rgbHandle.next_sequence;
			rgbHandle.sequence->enabled = true;
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
			  sequence->current_step_num = (step->step_type == REPEAT_STEP) ? (step->current_step_num + 1): step->next_step_num;
			  step->complete = false;
			  if(step->last_step && !sequence->loop){
				  sequence->enabled = false; //stop sequence if the last step finished and the sequence is not set to repeat
				  sequence->current_step_num = 0; //reset to first step
			  }
			  else if(step->last_step && sequence->loop){
				  sequence->current_step_num = 0; //reset to first step
			  }
		  }
	  }
	return;
}
//TIM4_IRQHandler
void TIM4_IRQHandler(void){
	HAL_TIM_IRQHandler(&RGBInterruptTimHandle);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	//run rgb led sequence routine
	if(htim->Instance == FUNC_RGB_INT_TIM_REG){
		FUNCTIONAL_RGB_LED_SequenceHandler();
	}
}

//called by Timer Initialization
void  HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim){
	if(htim->Instance == FUNC_RGB_INT_TIM_REG){
		RGB_LED_EnTimClk(htim->Instance);
		HAL_NVIC_SetPriority((IRQn_Type)(FUNC_RGB_INT_TIM_IRQ), 0x0F, 0x00);
		HAL_NVIC_EnableIRQ((IRQn_Type)(FUNC_RGB_INT_TIM_IRQ));
	}
}

//called by Timer DeInitialization
void  HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim){
	RGB_LED_DisTimClk(htim->Instance);
	HAL_NVIC_DisableIRQ((IRQn_Type)(FUNC_RGB_INT_TIM_IRQ));
}


