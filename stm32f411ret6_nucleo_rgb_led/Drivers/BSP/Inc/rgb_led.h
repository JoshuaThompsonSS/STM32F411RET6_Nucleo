/*
 * rgb_led.h
 *
 */

#ifndef _RGB_LED_H_
#define _RGB_LED_H_
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"


/*
 * ------Constants and Variable Declarations
*/

/*
 * RGB LED Count
 */

#define RGB_LED_COUNT	1 //make this 2 when we are ready to test 2 rgb leds
#define RGB_LED1_IDX	0

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
#define RGB_PWMFREQ_DFLT		1000 //1000 Hz (1KHz)

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
#define RGB1_RED_AF				GPIO_AF1_TIM2//GPIO_AF1_TIM1
#define RGB1_RED_PORT			GPIOA
#define RGB1_RED_PIN			GPIO_PIN_5//GPIO_PIN_9
#define RGB1_RED_TIM_CH			TIM_CHANNEL_1//TIM_CHANNEL_2
#define RGB1_RED_TIM_REG		TIM2//TIM1
#define RGB1_RED_TIMCLK_EN()	__TIM2_CLK_ENABLE()//__TIM1_CLK_ENABLE()

//GREEN
#define RGB1_GREEN_PORT			GPIOC
#define RGB1_GREEN_PIN			GPIO_PIN_8
#define RGB1_GREEN_TIM_CH		TIM_CHANNEL_3
#define RGB1_GREEN_TIM_REG		TIM3
#define RGB1_GREEN_AF			GPIO_AF2_TIM3
#define RGB1_GREEN_TIMCLK_EN()	__TIM3_CLK_ENABLE()
//BLUE
#define RGB1_BLUE_PORT			GPIOC //Booms are GPIOD
#define RGB1_BLUE_PIN			GPIO_PIN_9 //Booms are GPIO_PIN_13
#define RGB1_BLUE_TIM_CH		TIM_CHANNEL_4
#define RGB1_BLUE_TIM_REG		TIM3
#define RGB1_BLUE_AF			GPIO_AF2_TIM3
#define RGB1_BLUE_TIMCLK_EN()	__TIM3_CLK_ENABLE()

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
void RGB_LED_InitPWMConfig(rgb_led_color_conf_t * colorConfig);
void RGB_LED_InitLEDPwm(rgb_led_color_conf_t * colorConfig);
void RGB_LED_Start(int rgbnum);
void RGB_LED_StartLED(rgb_led_color_conf_t * colorConfig);
void RGB_LED_Stop(int rgbnum);
void RGB_LED_StopLED(rgb_led_color_conf_t * colorConfig);
void RGB_LED_DeInit(void);
void RGB_LED_Reset(void);
void RGB_LED_PowerDown(void);
void RGB_LED_SetConfig(void); //TODO: need config struct
void RGB_LED_ExitShutdown(void);
void RGB_LED_EnterShutdown(void);
void RGB_LED_SetColor(uint16_t red, uint16_t green, uint16_t blue);
void RGB_LED_ErrorHandler(void);
void RGB_LED_UpdateDutyCycle(uint32_t duty_cycle_percent, rgb_led_color_conf_t * colorConfig);
void RGB_LED_EnTimClk(TIM_TypeDef * TimRegx);
void RGB_LED_EnGpioClk(GPIO_TypeDef * port);





#endif /* _RGB_LED_H_ */
