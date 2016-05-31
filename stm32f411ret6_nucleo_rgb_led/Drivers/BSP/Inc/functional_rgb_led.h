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

/*
 * Structures / Variables
 */

//rgb function modes (logarithmic, exponential, linear)
typedef enum rgb_modes_t {RGB_LOG, RGB_EXP, RGB_LIN} rgb_modes_t;

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
	rgb_color_t color;
	rgb_color_t color_setpoint;
	uint32_t time; //millisec or micro?
	uint32_t duration; //millisec
	int number; //step number
	int prev_number;
	int next_number;
	bool complete;
	color_scales_t scales; //color diff / duration
	rgb_modes_t mode; //log, exponential, linear...etc
}rgb_led_step_t;


//rgb led step sequence
typedef struct rgb_led_sequence_t {
	int current_step_number;
	bool complete;
	rgb_led_step_t steps[MAX_RGB_STEPS];
}rgb_led_sequence_t;

/*
 * Sequences
 */

//ramp
rgb_led_sequence_t rampSequence;

//hold
rgb_led_sequence_t holdSequence;

//setpoint
rgb_led_sequence_t setpointSequence;

//repeat
rgb_led_sequence_t repeatSequence;

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
