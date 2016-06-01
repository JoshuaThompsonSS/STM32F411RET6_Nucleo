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

//RGB Colors
#define RGB_SOLID_RED			255
#define RGB_SOLID_GREEN			255
#define RGB_SOLID_BLUE			255
#define MIN_COLOR_DIFF			10

#define MAX_RGB_STEPS			10 //max steps in an rgb led sequence (ex: ramp, hold, repeat...etc)

//Service / Timer
//TIM REG Used for Interrupt (this is what runs the rgb led sequence)
#define FUNC_RGB_INT_TIM_REG	TIM4 //this must match the TIMx_IRQHandler definition in the function_rgb_led.c file
#define FUNC_RGB_INT_TIM_IRQ	TIM4_IRQn
#define STEP_TIME_PER_CYCLE		0.02 //(1.0/RGB_PWMFREQ_DFLT) //sec per timer interrupt -- TODO: need to define this appropriately

//Common sequence colors
#define BLACK_CLR				0
#define WHITE_CLR				1
#define RED_CLR					2
#define BLUE_CLR				3
#define GREEN_CLR				4
#define DEV_OFF_CLR				BLACK_CLR   //Device is Off
#define DEV_OK_CLR				WHITE_CLR	//Device is ok
#define DEV_CRTCL_CLR			RED_CLR     //Device critical state (battery, turning off, reset ..etc)
#define DEV_BLT_CLR				BLUE_CLR	//Device bluetooth process not complete

//RGB Step types
#define RAMP_STEP_TYPE			0
#define HOLD_STEP_TYPE			1
#define SETPOINT_STEP_TYPE		2
#define REPEAT_STEP_TYPE		3

//RGB Sequence / state types
#define ON_SEQ_TYPE				0 //on state
#define OFF_SEQ_TYPE			1 //off state
#define CRTCL_SEQ_TYPE			2 //critical battery state
#define CHRNG_SEQ_TYPE			3 //chargning state
#define CHRGD_SEQ_TYPE			4 //charged state
#define RST_SEQ_TYPE			5 //any reset state
#define BTCNTD_SEQ_TYPE			5 //bluetooth connected state
#define BTDISC_SEQ_TYPE			6 //bluetooth discoverable state
#define BTPR_SEQ_TYPE			7 //bluetooth pairing state
#define BTCNTG_SEQ_TYPE			8 //bluetooth connecting state
#define AXSH_SEQ_TYPE			9 //aux in shoe (1)
#define AXSH1_SEQ_TYPE			9 //aux in shoe1 not shoe2
#define FWUP_SEQ_TYPE			10 //firmware upgrade
#define FWUPCPLT_SEQ_TYPE		11 //firmware upgrade complete



/*
 * Structures / Variables
 */

//rgb function modes (logarithmic, exponential, linear)
typedef enum rgb_modes_t {RGB_LOG_MODE, RGB_EXP_MODE, RGB_LIN_MODE, RGB_NO_MODE} rgb_modes_t;
typedef enum rgb_step_type_t {RAMP_STEP=RAMP_STEP_TYPE, HOLD_STEP=HOLD_STEP_TYPE, SETPOINT_STEP=SETPOINT_STEP_TYPE, REPEAT_STEP=REPEAT_STEP_TYPE} rgb_step_type_t;
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



//rgb led step
//forward declaration of sequence type
typedef struct rgb_led_sequence_t rgb_led_sequence_t;

typedef struct rgb_led_step_t {
	bool last_step;
	rgb_step_type_t step_type;
	rgb_color_t color;
	rgb_color_t color_setpoint;
	float time; //millisec or micro?
	float duration; //millisec
	int current_step_num; //step number
	int next_step_num;
	bool complete;
	color_scales_t scales; //color diff / duration
	rgb_modes_t mode; //log, exponential, linear...etc
	void (*func_handler)(); //ptr to function that will run the step (for ex: FUNCTIONAL_RGB_LED_Ramp)
	rgb_led_sequence_t * parent_seq;
}rgb_led_step_t;


//rgb led step sequence
typedef struct rgb_led_sequence_t {
	int current_step_num;
	bool enabled;
	bool complete;
	bool restart;
	rgb_seq_type_t seq_type;
	rgb_led_step_t steps[MAX_RGB_STEPS];
}rgb_led_sequence_t;

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
	rgb_led_sequence_t * sequence;
}rgb_led_handle_t;

rgb_led_handle_t rgbHandle;

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
rgb_led_sequence_t fwUpgradeSequence;
rgb_led_sequence_t fwUpgradeCompleteSequence;

//Timer Interrupt that runs in background
TIM_HandleTypeDef RGBInterruptTimHandle;




/*
 * --------Function Declarations
*/
void FUNCTIONAL_REG_LED_InitHandle(void);
void FUNCTIONAL_RGB_LED_Init(void);
void FUNCTIONAL_RGB_LED_DeInit(void);
void FUNCTIONAL_RGB_LED_Start(void);
void FUNCTIONAL_RGB_LED_Stop(void);
void FUNCTIONAL_RGB_LED_Reset(void);
void FUNCTIONAL_RGB_LED_SetColor(rgb_color_t * color);
void FUNCTIONAL_RGB_LED_GetColor(rgb_color_t * color);
void FUNCTIONAL_RGB_LED_GetColorDiff(rgb_color_t * color1, rgb_color_t * color2, rgb_color_t * colorDelta);
//Steps
void FUNCTIONAL_RGB_LED_InitRampStep(rgb_led_step_t * step);
void FUNCTIONAL_RGB_LED_InitHoldStep(rgb_led_step_t * step);
void FUNCTIONAL_RGB_LED_InitSetpointStep(rgb_led_step_t * step);
void FUNCTIONAL_RGB_LED_InitRepeatStep(rgb_led_step_t * step);
//Sequences
void FUNCTIONAL_RGB_LED_InitOnSeq(rgb_led_sequence_t * sequence);
void FUNCTIONAL_RGB_LED_InitOffSeq(void);
void FUNCTIONAL_RGB_LED_InitCriticalSeq(void);
void FUNCTIONAL_RGB_LED_InitBTConnectedSeq(void);
void FUNCTIONAL_RGB_LED_InitBTDiscoverableSeq(void);
void FUNCTIONAL_RGB_LED_InitBTPairingSeq(void);
void FUNCTIONAL_RGB_LED_InitBTConnectingSeq(void);
void FUNCTIONAL_RGB_LED_InitAUXInShoeSeq(void);
void FUNCTIONAL_RGB_LED_InitAUXInShoe1Seq(void);
void FUNCTIONAL_RGB_LED_InitFWUpgradeSeq(void);
void FUNCTIONAL_RGB_LED_InitFWUpgradeCompleteSeq(void);


//Step functions to create specific RGB LED sequences
void FUNCTIONAL_RGB_LED_Ramp(rgb_led_step_t * step);
void FUNCTIONAL_RGB_LED_Hold(rgb_led_step_t * step);
void FUNCTIONAL_RGB_LED_Setpoint(rgb_led_step_t * step);
void FUNCTIONAL_RGB_LED_Repeat(rgb_led_step_t * step);

//Sequence routine service - should be called at least every 50 ms

void FUNCTIONAL_RGB_LED_InitInterruptTimer(void);
void FUNCTIONAL_RGB_LED_SequenceHandler(void);




#endif /* _RGB_LED_H_ */
