/*
 * RGB_LED.h
 *
 */

#ifndef _RGB_LED_H_
#define _RGB_LED_H_
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"


/* LED PINS
 * Add PIN sets here as long as they can be mapped to a TIMER (ex: TIM2_CH1)
 * */
#define AF_ERROR			0xFF
/* RGB LED 1 Config Data */

//RGB LED 1 PIN Mapping - so far we are just testing with 1 RGB LED but can add more assuming we have enough TIMER ports available
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


void RGB_LED_Init(void);
void RGB_LED_InitLEDConfig(TIM_HandleTypeDef * TimHandlePtr, TIM_OC_InitTypeDef * PwmConfigPtr, TIM_TypeDef * TimReg, uint32_t TimChannel);
void RGB_LED_InitLEDPwm(GPIO_InitTypeDef * GPIO_InitStruct, GPIO_TypeDef * LED_PORT, uint16_t LED_PIN);
void RGB_LED_Start(void);
void RGB_LED_StartLED(TIM_HandleTypeDef * TimHandlePtr, uint32_t TimChannel);
void RGB_LED_Stop(void);
void RGB_LED_StopLED(TIM_HandleTypeDef * TimHandlePtr, uint32_t TimChannel);
void RGB_LED_DeInit(void);
void RGB_LED_Reset(void);
void RGB_LED_PowerDown(void);
void RGB_LED_SetConfig(void); //TODO: need config struct
void RGB_LED_ExitShutdown(void);
void RGB_LED_EnterShutdown(void);
void RGB_LED_SetColor(uint16_t red, uint16_t green, uint16_t blue);
void RGB_LED_ErrorHandler(void);
uint8_t RGB_LED_GetLED_AF(GPIO_TypeDef * LED_PORT, uint16_t LED_PIN);

//RGB LED 1 TIM Handlers for PWM Generator
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



#endif /* _RGB_LED_H_ */
