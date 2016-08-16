/*
 * tlc_rgb_led.h - driver for the TLC59116 led driver from TI
 *
 */

#ifndef _TLC_RGB_LED_H_
#define _TLC_RGB_LED_H_
#include "stm32f4xx_hal.h"
#include "tlc59116.h"


/*
 * ------Constants and Variable Declarations
*/

/* Convert RGB 255 color to 0 - 100% duty cycle
*
* RGB_PWM_OFFSET_x can be used when RGB LED sink is Positive Voltage instead of Ground
* When sink is positive voltage then 100% pwm results in 0 brightness
* so adding a -100% offset makes is 100% brightness
*
* ex: blue value of 240 -> pwm output = 100% + 240*(-100%/255) = 5% pwm output which is almost 100% brightness
*     if sink is positive voltage
*     if sink is groudn then just do: 0% + 240*(100%/255) = 94%
*/
#define RGB_TO_PWM_SCALE_RED	(100.0/255.0)//(-100.0/255.0)
#define RGB_PWM_OFFSET_RED		0//100.0
#define RGB_TO_PWM_SCALE_GREEN	(100.0/255.0)//(-100.0/255.0)
#define RGB_PWM_OFFSET_GREEN	0//100.0
#define RGB_TO_PWM_SCALE_BLUE	(100.0/255.0)//(-100.0/255.0)
#define RGB_PWM_OFFSET_BLUE		0//100.0

/*
 * RGB LED Count
 */

#define TLC_RGB_LED_COUNT	5 //there are currently 5 rgb leds connected to the TLC59116 led driver
#define FUNC_TLC_RGB_LED_NUM	0 //functional RGB LED number
//#define DEC_TLC_RGB_LED_NUM		1 //decorative RGB LED number

/* LED PINS
 * Add PIN sets here as long as they can be mapped to a TIMER (ex: TIM2_CH1)
 */

//Alternate Function number when user tries to assing pin that does not have an AF assigned
#define AF_ERROR			0xFF

//Use these defaults for the STM32F4 TIM PWM generation
#define RGB_PRESCALER_DFLT	1000 //timer_freq = system clock / (prescaler + 1) -- ex: timer_freq = 84MHz / (1000 + 1) = 84KHz

/* PWM Frequency = 1 / Period
 * Period = Period(Cycles)  / timer_freq -- Period(cycles) is the value used in the TIM register config
 * So Period(Cycles) = Period * timer_freq
 */
#define RGB_PWMFREQ_DFLT	50 //50 - 200 Hz is good (don't go over 500 Hz or wont work correctly)
#define RGB_PWMTIM_PERIOD	(1000U - 1U)
/* Duty_Cycle
 * Pwm Duty Cycle = 100% * Pulse(Cycles) / Period(Cycles)
 * So Pulse(Cycles) = Duty_Cycle * Period(Cycles) -- use Pulse value in the TIM register PWM Config
 */
#define RGB_PWMDUTY_DFLT	50 //50% -- this is the percentage of time that the GPIO output is HIGH out of the total PWM period

//RGB LED mapping
#define FUNCTIONAL_RGB_NUM			0
#define FUNCTIONAL_RGB_RED_CH		0
#define FUNCTIONAL_RGB_GREEN_CH		0
#define FUNCTIONAL_RGB_BLUE_CH		0

#define DECORATIVE_RGB1_NUM			1
#define DECORATIVE_RGB1_RED_CH		1
#define DECORATIVE_RGB1_GREEN_CH	1
#define DECORATIVE_RGB1_BLUE_CH		1


#define DECORATIVE_RGB2_NUM			2
#define DECORATIVE_RGB2_RED_CH		1
#define DECORATIVE_RGB2_GREEN_CH	1
#define DECORATIVE_RGB2_BLUE_CH		1

#define DECORATIVE_RGB3_NUM			3
#define DECORATIVE_RGB3_RED_CH		1
#define DECORATIVE_RGB3_GREEN_CH	1
#define DECORATIVE_RGB3_BLUE_CH		1

#define DECORATIVE_RGB4_NUM			4
#define DECORATIVE_RGB4_RED_CH		1
#define DECORATIVE_RGB4_GREEN_CH	1
#define DECORATIVE_RGB4_BLUE_CH		1


/*
 * RGB LED Structure and Collections
 */
//color struct
typedef struct rgb_color_t {
	int red; //rgb range: 0 - 255
	int green;
	int blue;
} rgb_color_t;

//duty cycle struct
typedef struct rgb_duty_cyle_t {
	float red; //%0-100%
	float green;
	float blue;
} rgb_duty_cycle_t;


//handles individual led output
typedef struct TLC_LED_handler_t {
	uint8_t	   		channel;
	uint8_t 		pwm;

} TLC_LED_handler_t;

//handles 3 led outputs that make up an rgb led
typedef struct TLC_RGB_LED_handler_t {
	TLC_LED_handler_t red;
	TLC_LED_handler_t green;
	TLC_LED_handler_t blue;
	rgb_color_t color;
} TLC_RGB_LED_handler_t;


/*
 * --------Function Declarations
 */
void TLC_RGB_LED_InitConfigs(void);
void TLC_RGB_LED_Init(int rgbnum);
void TLC_RGB_LED_DeInit(int rgbnum);
void TLC_RGB_LED_Reset(int rgbnum);
void TLC_RGB_LED_Start(int rgbnum);
void TLC_RGB_LED_StartLED(int rgbnum);
void TLC_RGB_LED_Stop(int rgbnum);
void TLC_RGB_LED_StopLED(int rgbnum);
void TLC_RGB_LED_PowerDown(int rgbnum);
void TLC_RGB_LED_ExitShutdown(void);
void TLC_RGB_LED_EnterShutdown(void);
void TLC_RGB_LED_SetColor(int rgbnum, rgb_color_t * color);
void TLC_RGB_LED_GetColor(int rgbnum, rgb_color_t * color);
int TLC_RGB_LED_Round(float value);
void TLC_RGB_LED_SetDutyCycle(rgb_duty_cycle_t * rgb_duty_cycle, int rgbnum);
void TLC_RGB_LED_UpdateDutyCycle(int rgbnum);
void TLC_RGB_LED_ErrorHandler(void);


#endif /* _TLC_RGB_LED_H_ */
