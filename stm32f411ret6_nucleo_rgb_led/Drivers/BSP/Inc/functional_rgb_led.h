/*
 * functional_rgb_led.h
 *
 */

#ifndef _FUNCTIONAL_RGB_LED_H_
#define _FUNCTIONAL_RGB_LED_H_
#include "rgb_led.h"
#include <stdbool.h>

/*
 * ------Constants / Definitions
*/
#define MAX_RGB_STEPS		10 //max steps in an rgb led sequence (ex: ramp, hold, repeat...etc)

//Service / Timer
#define SERVICE_TIMER_REG	TIM2
#define SERVICE_TIMER_CH	TIM_CH_2

//Common sequence colors
#define BLACK_CLR			0
#define WHITE_CLR			1
#define RED_CLR				2
#define BLUE_CLR			3
#define GREEN_CLR			4
#define DEV_OFF_CLR			BLACK_CLR   //Device is Off
#define DEV_OK_CLR			WHITE_CLR	//Device is ok
#define DEV_CRTCL_CLR		RED_CLR     //Device critical state (battery, turning off, reset ..etc)
#define DEV_BLT_CLR			BLUE_CLR	//Device bluetooth process not complete

//RGB Step types
#define RAMP_STEP_TYPE			0
#define HOLD_STEP_TYPE			1
#define SETPOINT_STEP_TYPE		2
#define REPEAT_STEP_TYPE			3

//RGB Sequence / state types
#define ON_SEQ_TYPE			0 //on state
#define OFF_SEQ_TYPE			1 //off state
#define CRTCL_SEQ_TYPE		2 //critical battery state
#define CHRNG_SEQ_TYPE		3 //chargning state
#define CHRGD_SEQ_TYPE		4 //charged state
#define RST_SEQ_TYPE			5 //any reset state
#define BTCNTD_SEQ_TYPE		5 //bluetooth connected state
#define BTDISC_SEQ_TYPE		6 //bluetooth discoverable state
#define BTPR_SEQ_TYPE		7 //bluetooth pairing state
#define BTCNTG_SEQ_TYPE		8 //bluetooth connecting state
#define AXSH_SEQ_TYPE		9 //aux in shoe (1)
#define AXSH1_SEQ_TYPE		9 //aux in shoe1 not shoe2
#define FWUP_SEQ_TYPE		10 //firmware upgrade
#define FWUPCPLT_SEQ_TYPE	11 //firmware upgrade complete



/*
 * Structures / Variables
 */

//rgb function modes (logarithmic, exponential, linear)
typedef enum rgb_modes_t {RGB_LOG, RGB_EXP, RGB_LIN} rgb_modes_t;
typedef enum rgb_step_type_t {RAMP=RAMP_STEP_TYPE, HOLD=HOLD_STEP_TYPE, SETPOINT=SETPOINT_STEP_TYPE, REPEAT=REPEAT_STEP_TYPE} rgb_step_type_t;
typedef enum rgb_state_colors_t {BLACK=BLACK_CLR, WHITE=WHITE_CLR, RED=RED_CLR, BLUE=BLUE_CLR, GREEN=GREEN_CLR} rgb_state_colors_t;
typedef enum rgb_seq_type_t {ON=ON_SEQ_TYPE, OFF=OFF_SEQ_TYPE, CRITICAL=ON_SEQ_TYPE, CHARGING=CHRNG_SEQ_TYPE, CHARGED=CHRGD_SEQ_TYPE,
								RESET_SHOE=RST_SEQ_TYPE, BT_CONNECTED=BTCNTD_SEQ_TYPE, BT_DISCOVERABLE=BTDISC_SEQ_TYPE, BT_PAIRING=BTPR_SEQ_TYPE,
								BT_CONNECTING=BTCNTG_SEQ_TYPE, AUX_IN_SHOE=AXSH_SEQ_TYPE, AUX_IN_SHOE1=AXSH1_SEQ_TYPE, FW_UPGRADE=FWUP_SEQ_TYPE,
								FW_UPGRADE_COMPLETE=FWUPCPLT_SEQ_TYPE
}rgb_seq_type_t;

//rgb color ramp scale
//ex: red_scale = (color setpoint - current color) / duration
typedef struct color_scales_t{
	float red;
	float green;
	float blue;
}color_scales_t;

//rgb led handle
typedef struct rgb_led_handle_t {
	void (*set_color)();
	void (*get_color)();
	void (*get_color_diff)();
	void (*init)();
	void (*de_init)();
	void (*start)();
	void (*stop)();
	void (*reset)();
}rgb_led_handle_t;

//rgb led step
typedef struct rgb_led_step_t {
	rgb_step_type_t step_type;
	rgb_color_t color_measured;
	rgb_color_t color_setpoint;
	uint32_t time; //millisec or micro?
	uint32_t duration; //millisec
	int number; //step number
	int next_number;
	bool complete;
	color_scales_t scales; //color diff / duration
	rgb_modes_t mode; //log, exponential, linear...etc
	void (*func_handler)(); //ptr to function that will run the step (for ex: FUNCTIONAL_RGB_LED_Ramp)
}rgb_led_step_t;


//rgb led step sequence
typedef struct rgb_led_sequence_t {
	int current_step_number;
	bool complete;
	bool restart;
	rgb_seq_type_t seq_type;
	rgb_led_step_t steps[MAX_RGB_STEPS];
}rgb_led_sequence_t;



/*
 * Common Sequences - based on 'Functional LED Behavior' documented in Confluence - created by Eddie Borjas
 * 					- Sequences of led steps that make up an entire blinking routine (ramp, hold, repeat...etc)
 */
rgb_led_sequence_t onSequence;
rgb_led_sequence_t offSequence;
rgb_led_sequence_t criticalBattSequence;
rgb_led_sequence_t chargingSequence;
rgb_led_sequence_t chargedSequence;
rgb_led_sequence_t resetSequence;
rgb_led_sequence_t btConnectedSequence;
rgb_led_sequence_t btDiscoverableSequence;
rgb_led_sequence_t btPairingSequence;
rgb_led_sequence_t btConnectingSequence;
rgb_led_sequence_t auxInShoeSequence;
rgb_led_sequence_t auxInShoe1Sequence;
rgb_led_sequence_t Sequence;
rgb_led_sequence_t fwUpgradeSequence;
rgb_led_sequence_t fwUpgradeCompleteSequence;




/*
 * --------Function Declarations
*/
void FUNCTIONAL_RGB_LED_Init(void);
void FUNCTIONAL_RGB_LED_InitRampSeq(void);
void FUNCTIONAL_RGB_LED_InitHoldSeq(void);
void FUNCTIONAL_RGB_LED_InitSetpointSeq(void);
void FUNCTIONAL_RGB_LED_InitRepeatSeq(void);
void FUNCTIONAL_RGB_LED_Init(void);
void FUNCTIONAL_RGB_LED_Start(void);
void FUNCTIONAL_RGB_LED_Stop(void);
void FUNCTIONAL_RGB_LED_DeInit(void);
void FUNCTIONAL_RGB_LED_Reset(void);
void FUNCTIONAL_RGB_LED_SetColor(rgb_color_t * color);
void FUNCTIONAL_RGB_LED_GetColor(rgb_color_t * color);
void FUNCTIONAL_RGB_LED_GetColorDiff(rgb_color_t * color1, rgb_color_t * color2, rgb_color_t * colorDelta);

//Step functions to create specific RGB LED sequences
void FUNCTIONAL_RGB_LED_Ramp(void);
void FUNCTIONAL_RGB_LED_Hold(void);
void FUNCTIONAL_RGB_LED_Setpoint(void);
void FUNCTIONAL_RGB_LED_Repeat(void);

//Sequence routine service - should be called at least every 100 ms by SysTick_Handler in stm32f4xx_it.c
void FUNCTIONAL_RGB_LED_IRQHandler(void);




#endif /* _RGB_LED_H_ */
