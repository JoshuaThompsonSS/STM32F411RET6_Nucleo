/*
 * rgb_led.h
 *
 */

#ifndef _RGB_LED_H_
#define _RGB_LED_H_
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "rgb.h"


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

#define RGB_LED_COUNT	1 //make this 2 when we are ready to test 2 rgb leds
#define FUNC_RGB_LED_NUM	0 //functional RGB LED number
//#define DEC_RGB_LED_NUM		1 //decorative RGB LED number

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

/* Duty_Cycle
 * Pwm Duty Cycle = 100% * Pulse(Cycles) / Period(Cycles)
 * So Pulse(Cycles) = Duty_Cycle * Period(Cycles) -- use Pulse value in the TIM register PWM Config
 */
#define RGB_PWMDUTY_DFLT	50 //50% -- this is the percentage of time that the GPIO output is HIGH out of the total PWM period
/* RGB LED 1 Config Data */

/*
 * RGB LED 1 PIN Mapping - so far we are just testing with 1 RGB LED but can add more assuming we have enough TIMER ports available
 */
//RED
#define RGB1_RED_AF				GPIO_AF1_TIM2 //GPIO_AF1_TIM1
#define RGB1_RED_PORT			GPIOA
#define RGB1_RED_PIN			GPIO_PIN_5//GPIO_PIN_9
#define RGB1_RED_TIM_CH			TIM_CHANNEL_1 //TIM_CHANNEL_2
#define RGB1_RED_TIM_REG		TIM2//TIM1
#define RGB1_RED_TIMCLK_EN()	__TIM2_CLK_ENABLE()//__TIM1_CLK_ENABLE()
#define RGB1_RED_PER_DFLT		8400 //period default of pwm timer
#define RGB1_RED_PS_DFLT		1000 //prescaler default of pwm timer
#define RGB1_RED_PULSE_DFLT		4000 //pulse default of pwm timer

//GREEN
#define RGB1_GREEN_PORT			GPIOC
#define RGB1_GREEN_PIN			GPIO_PIN_8
#define RGB1_GREEN_TIM_CH		TIM_CHANNEL_3
#define RGB1_GREEN_TIM_REG		TIM3
#define RGB1_GREEN_AF			GPIO_AF2_TIM3
#define RGB1_GREEN_TIMCLK_EN()	__TIM3_CLK_ENABLE()
#define RGB1_GREEN_PER_DFLT		8400 //period default of pwm timer
#define RGB1_GREEN_PS_DFLT		1000 //prescaler default of pwm timer
#define RGB1_GREEN_PULSE_DFLT	4000 //pulse default of pwm timer
//BLUE
#define RGB1_BLUE_PORT			GPIOC //Booms are GPIOD
#define RGB1_BLUE_PIN			GPIO_PIN_9 //Booms are GPIO_PIN_13
#define RGB1_BLUE_TIM_CH		TIM_CHANNEL_4
#define RGB1_BLUE_TIM_REG		TIM3
#define RGB1_BLUE_AF			GPIO_AF2_TIM3
#define RGB1_BLUE_TIMCLK_EN()	__TIM3_CLK_ENABLE()
#define RGB1_BLUE_PER_DFLT		8400 //period default of pwm timer
#define RGB1_BLUE_PS_DFLT		1000 //prescaler default of pwm timer
#define RGB1_BLUE_PULSE_DFLT	4000 //pulse default of pwm timer

/*
 *RGB LED TIMER PWM Config Handlers and Structures
 */
//RGB LED 1
//RED
TIM_HandleTypeDef RGB1_RedTimHandle;
TIM_OC_InitTypeDef RGB1_RedPwmConfig;
GPIO_InitTypeDef  GPIO_RGB1_RedInitStruct;

//GREEN
TIM_HandleTypeDef RGB1_GreenTimHandle;
TIM_OC_InitTypeDef RGB1_GreenPwmConfig;
GPIO_InitTypeDef  GPIO_RGB1_GreenInitStruct;
//BLUE
TIM_HandleTypeDef RGB1_BlueTimHandle;
TIM_OC_InitTypeDef RGB1_BluePwmConfig;
GPIO_InitTypeDef  GPIO_RGB1_BlueInitStruct;

/*
 * RGB LED Color Config Structure and Collections
 */

//holds individual color config
typedef struct rgb_led_color_conf_t {
	GPIO_TypeDef * 		port; //GPIOx
	uint16_t	   		pin; //GPIOx_Pinx
	uint32_t	   		timChannel; //TIMx_Channelx
	TIM_TypeDef *  		timReg; //TIMx
	uint8_t		   		afNum; //Alternate Function Number
	TIM_HandleTypeDef 	timHandle;
	TIM_OC_InitTypeDef 	pwmConfig;
	GPIO_InitTypeDef  	gpioInitStruct;

} rgb_led_color_conf_t;

//hold config of red, green and blue configs
typedef struct rgb_led_conf_t {
	rgb_led_color_conf_t red;
	rgb_led_color_conf_t green;
	rgb_led_color_conf_t blue;
} rgb_led_conf_t;

//array of rgb led configs
rgb_led_conf_t RgbLedConfigs[RGB_LED_COUNT];

/*
 * --------Function Declarations
 */
void RGB_LED_InitConfigs(int rgbnum);
void RGB_LED_Init(int rgbnum);
void RGB_LED_DeInitLEDGpio(rgb_led_color_conf_t * colorConfig);
void RGB_LED_Reset(int rgbnum);
void RGB_LED_InitPWMConfig(rgb_led_color_conf_t * colorConfig);
void RGB_LED_DeInitPWMConfig(rgb_led_color_conf_t * colorConfig);
void RGB_LED_InitLEDGpio(rgb_led_color_conf_t * colorConfig);
void RGB_LED_Start(int rgbnum);
void RGB_LED_StartLED(rgb_led_color_conf_t * colorConfig);
void RGB_LED_Stop(int rgbnum);
void RGB_LED_StopLED(rgb_led_color_conf_t * colorConfig);
void RGB_LED_DeInit(int rgbnum);
void RGB_LED_PowerDown(int rgbnum);
void RGB_LED_ExitShutdown(void);
void RGB_LED_EnterShutdown(void);
void RGB_LED_SetColor(int rgbnum, rgb_color_t * color);
int RGB_LED_Round(float value);
void RGB_LED_GetColor(int rgbnum, rgb_color_t * color);
void RGB_LED_ErrorHandler(void);
void RGB_LED_SetDutyCycle(float duty_cycle_percent, rgb_led_color_conf_t * colorConfig);
float RGB_LED_GetDutyCycle(rgb_led_color_conf_t * colorConfig);
void RGB_LED_EnTimClk(TIM_TypeDef * TimRegx);
void RGB_LED_DisTimClk(TIM_TypeDef * TimRegx);
void RGB_LED_EnGpioClk(GPIO_TypeDef * port);





#endif /* _RGB_LED_H_ */
