/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_uart.h"
#include "functional_rgb_led.h"
#include "rgb_led.h"
#include "tlc_rgb_led.h"
//#include "i2c.h"
#include "tlc59116.h"
#include <math.h>
#include "stdlib.h"
#include "string.h"



/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
#define maxStr 100

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef TimHandle;
TIM_OC_InitTypeDef PwmConfig;
TIM_HandleTypeDef LEDTimHandle;
GPIO_InitTypeDef  GPIO_InitStruct;

UART_InitTypeDef UART_InitStructure;
UART_InitTypeDef UART1_InitStructure;
UART_HandleTypeDef UART_Handle;
UART_HandleTypeDef UART1_Handle;
rgb_seq_type_t rgbLedOn = RGBSEQ_ON;
rgb_seq_type_t rgbLedOff = RGBSEQ_OFF;
char *msg = "Hello Nucleo Fun!\n\r";
float upDur, downDur, holdDur;
int modeNum;
int seqNum = 0;

//Power mode testing variables
int global_a = 0, global_b = 0;
extern tlc59116_register_controller_t tlcHandler;;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */


/* Private function prototypes -----------------------------------------------*/
int getNumStr(char * buffer, char * nums, int startIndex, int maxChars);
int convertStrToMs(char * numStr, int powerCount);
void SystemClock_Config(void);
void InitRGBCmd(void);
void CmdLine(void);
void InitRedLed(void);
void StartRedLed(void);
void StopRedLed(void);

/* USER CODE BEGIN PFP */


/* Function prototypes -----------------------------------------------*/

void Init_USART(void){
//http://www.carminenoviello.com/2015/03/02/how-to-use-stm32-nucleo-serial-port/
// Initialize USART - TODO: make sure no hardware flow control

UART_InitStructure.BaudRate = 9600;
UART_InitStructure.WordLength = UART_WORDLENGTH_8B;
UART_InitStructure.StopBits = UART_STOPBITS_1;
UART_InitStructure.Parity = UART_PARITY_NONE;
UART_InitStructure.Mode = UART_MODE_TX_RX;
UART_InitStructure.HwFlowCtl = UART_HWCONTROL_NONE;
/* Configure USART */
UART_Handle.Instance = USART2;
UART_Handle.Init = UART_InitStructure;
/* Init and Enable the USART */
HAL_UART_Init(&UART_Handle);

//__HAL_USART_ENABLE(&USART_Handle);
}

void Init_USART1(void){
	UART1_InitStructure.BaudRate = 9600;
	UART1_InitStructure.WordLength = UART_WORDLENGTH_8B;
	UART1_InitStructure.StopBits = UART_STOPBITS_1;
	UART1_InitStructure.Parity = UART_PARITY_EVEN;
	UART1_InitStructure.Mode = UART_MODE_TX_RX;
	UART1_InitStructure.HwFlowCtl = UART_HWCONTROL_NONE;
	/* Configure USART */
	UART1_Handle.Instance = USART1;
	UART1_Handle.Init = UART1_InitStructure;
	/* Init and Enable the USART */
	HAL_UART_Init(&UART1_Handle);

}

//Initialize timer used to generate interrupt
void initTimInterrupt(void){
	//Make sure HAL_TIM_Base_MsInit() code is implemented (GPIO led en, timer clock enable...etc)
	//to use the Timer to generate a simple time base

	LEDTimHandle.Instance = TIM2;
	uint32_t timer_freq = HAL_RCC_GetSysClockFreq() / RGB_PRESCALER_DFLT;
	int32_t timer_period = timer_freq / 1; //RGB_PWMFREQ_DFLT;
	LEDTimHandle.Init.Period = timer_period;
	LEDTimHandle.Init.Prescaler = RGB_PRESCALER_DFLT;
	LEDTimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	LEDTimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	LEDTimHandle.Init.RepetitionCounter = 0;

	HAL_TIM_Base_Init(&LEDTimHandle);

	//activate TIM / start with interrupt
	HAL_TIM_Base_Start_IT(&LEDTimHandle);

}

//Initialize the board debug led as output
void initLED(void){
	//enable GPIOA peripheral clock
	__HAL_RCC_GPIOA_CLK_ENABLE();
	//enable GPIOA pin 5 as output

	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

//Initialize button as interrupt
void initButtonInterrupt(void){
	//enable GPIOA peripheral clock
	__HAL_RCC_GPIOC_CLK_ENABLE();
	//configure GPIOA pin 13 as interrupt with falling edge sensitivity
	GPIO_InitTypeDef  GPIO_InitStruct;
	GPIO_InitStruct.Mode =  GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	//interrupt enable EXTI line 13
	/* Enable and set Button EXTI Interrupt to the lowest priority */
	HAL_NVIC_SetPriority((IRQn_Type)(EXTI15_10_IRQn), 0x0F, 0x00);
	HAL_NVIC_EnableIRQ((IRQn_Type)(EXTI15_10_IRQn));

}

//Initialize PA0 as interrupt
void initWakeupInterrupt(void){

	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);

}

//initialize timer used for pwm (timer 2 ch 2)
void initLEDPwm(void){
	//enable GPIOA peripheral clock
	//
		__HAL_RCC_GPIOA_CLK_ENABLE(); //RCC->AHB1ENR |= 1;
		//enable GPIOA pin 5 as output

		GPIO_InitTypeDef  GPIO_InitStruct;
		GPIO_InitStruct.Pin = GPIO_PIN_5;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;//GPIO_PULLDOWN;
		GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
		GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}


void initPwmTimer(void){
	//gpio init

	//initLEDPwm(); called by the HAL_TIM_PWM_MspInit() defined at end of file
	//timer init

	TimHandle.Instance = TIM2;
	//timer freq = 84MHz / (prescaler + )
	TimHandle.Init.Period = 8400; //period of 1 hz if prescaler = 1000
	TimHandle.Init.Prescaler = 1000;
	//pwm freq = period / timer_freq
	TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	TimHandle.Init.RepetitionCounter = 0;

	//configure TIM mode for pwm function generator
	//HAL_TIM_PWM_MspInit(&TimHandle);
	HAL_TIM_PWM_Init(&TimHandle);
	//enable timer clocks
	//__TIM2_CLK_ENABLE(); called in HAL_TIM_PWM_MspInit()


	PwmConfig.OCMode = TIM_OCMODE_PWM1;
	PwmConfig.OCFastMode = TIM_OCFAST_ENABLE;
	PwmConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
	PwmConfig.Pulse = 4000;
	HAL_TIM_PWM_ConfigChannel(&TimHandle, &PwmConfig, TIM_CHANNEL_1);


	//Activate the TIM peripheral
	HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1); //start pwm on port b ch 3

}



//wait function
void wait_sec(float sec){
	HAL_Delay(sec*1000);
}

//toggle led
void toggleLED(void){
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}

void blink_led(int c){
	for(int i = 0; i<c; i++){
		toggleLED();
		wait_sec(1);
		toggleLED();
		wait_sec(1);
	}
}
void Custom_HAL_PWR_EnterSTOPMode(uint32_t Regulator, uint8_t STOPEntry)
{
  /* Check the parameters */
  assert_param(IS_PWR_REGULATOR(Regulator));
  assert_param(IS_PWR_STOP_ENTRY(STOPEntry));

  /* Select the regulator state in Stop mode: Set PDDS and LPDS bits according to PWR_Regulator value */
  MODIFY_REG(PWR->CR, (PWR_CR_PDDS | PWR_CR_LPDS), Regulator);

  /* Set SLEEPDEEP bit of Cortex System Control Register */
  SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));

  /* Select Stop mode entry --------------------------------------------------*/
  if(STOPEntry == PWR_STOPENTRY_WFI)
  {
    /* Request Wait For Interrupt */
    __WFI();
  }
  else
  {
    /* Request Wait For Event */
    __SEV();
    __WFE();
    __WFE();
  }
  /* Reset SLEEPDEEP bit of Cortex System Control Register */
  CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));
}

void stopModeTest(void){
	/*
	 * Make sure peripherals clocks off, main clock on, SRAM and registers saved, variables saved
	 *
	 */
	/* Initialize LED*/
	 initLED();
	 //toggleLED(); //turn led pa5 to HIGH state
	 GPIO_TypeDef  * GPIOx = GPIOA;
	 uint32_t modera = GPIOx->MODER;
	 uint32_t idr = GPIOx->IDR;
	 uint32_t bsrr = GPIOx->BSRR;
	 uint32_t lckr = GPIOx->LCKR;
	 uint32_t odr = GPIOx->ODR;
	 uint32_t osspeed = GPIOx->OSPEEDR;
	 uint32_t otype = GPIOx->OTYPER;
	 uint32_t pupdr = GPIOx->PUPDR;
	/* Initialize Button Interrupt */
	initButtonInterrupt();

	//variables to check if saved
	int a = 0, b = 0;

	//Init stop mode test variables
	a = 5; b = 6;
	global_a = 5; global_b = 6;

	HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI); //PWR_LOWPOWERREGULATOR_ON
	/* Initialize PWM for Functional RGB LED only if variables saved*/
	//make sure GPIOA register did not change
	int c = 0;
	bool saved = false;
	bool saved_reg = false;
	 saved_reg = (GPIOA->MODER == modera);
	 saved = saved_reg;
	 c++;
	 //if(!saved_reg){blink_led(c); wait_sec(5);}

	 saved_reg = (GPIOA->IDR == idr);
	 saved &= saved_reg;
	 c++;
	 //if(!saved_reg){blink_led(c); wait_sec(5);}

	 saved_reg = (GPIOA->BSRR == bsrr);
	 saved &= saved_reg;
	 c++;
	 //if(!saved_reg){blink_led(c); wait_sec(5);}

	 saved_reg = (GPIOA->LCKR == lckr);
	 saved &= saved_reg;
	 c++;
	 //if(!saved_reg){blink_led(c); wait_sec(5);}

	 saved_reg = (GPIOA->ODR == odr);
	 saved &= saved_reg;
	 c++;
	 //if(!saved_reg){blink_led(c); wait_sec(5);}

	 saved_reg = (GPIOA->OSPEEDR == osspeed);
	 saved &= saved_reg;
	 c++;
	 //if(!saved){blink_led(c); wait_sec(5);}

	 saved_reg = (GPIOA->OTYPER == otype);
	 saved &= saved_reg;
	 c++;
	 //if(!saved_reg){blink_led(c); wait_sec(5);}

	 saved_reg = (GPIOA->PUPDR == pupdr);
	 saved &= saved_reg;
	 c++;
	 //if(!saved_reg){blink_led(c); wait_sec(5);}

	//LED sequence should run only if registers and variables were saved after STOP mode exits
	if(a==5 && b==6 && global_a==5 && global_b ==6 && saved){
		InitRGBCmd(); //init uart, rgb led driver and services
	}

}




void sleepModeTest(void){
	/*
	 * Make sure peripherals clocks off, main clock on, SRAM and registers saved, variables saved
	 *
	 */
	/* Initialize LED*/
	 initLED();
	 GPIO_TypeDef  * GPIOx = GPIOA;
	 uint32_t modera = GPIOx->MODER;
	 uint32_t idr = GPIOx->IDR;
	 uint32_t bsrr = GPIOx->BSRR;
	 uint32_t lckr = GPIOx->LCKR;
	 uint32_t odr = GPIOx->ODR;
	 uint32_t osspeed = GPIOx->OSPEEDR;
	 uint32_t otype = GPIOx->OTYPER;
	 uint32_t pupdr = GPIOx->PUPDR;
	/* Initialize Button Interrupt */
	initButtonInterrupt();

	//variables to check if saved
	int a = 0, b = 0;

	//Init stop mode test variables
	a = 5; b = 6;
	global_a = 5; global_b = 6;

	//Enter sleep mode
	HAL_SuspendTick(); //need to stop the systick timer interrupt (since it will wake up the mcu from sleep)
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	HAL_ResumeTick();

	//make sure GPIOA register did not change
	int c = 0;
	bool saved = false;
	bool saved_reg = false;
	 saved_reg = (GPIOA->MODER == modera);
	 saved = saved_reg;
	 c++;
	 //if(!saved_reg){blink_led(c); wait_sec(5);}

	 saved_reg = (GPIOA->IDR == idr);
	 saved &= saved_reg;
	 c++;
	 //if(!saved_reg){blink_led(c); wait_sec(5);}

	 saved_reg = (GPIOA->BSRR == bsrr);
	 saved &= saved_reg;
	 c++;
	 //if(!saved_reg){blink_led(c); wait_sec(5);}

	 saved_reg = (GPIOA->LCKR == lckr);
	 saved &= saved_reg;
	 c++;
	 //if(!saved_reg){blink_led(c); wait_sec(5);}

	 saved_reg = (GPIOA->ODR == odr);
	 saved &= saved_reg;
	 c++;
	 //if(!saved_reg){blink_led(c); wait_sec(5);}

	 saved_reg = (GPIOA->OSPEEDR == osspeed);
	 saved &= saved_reg;
	 c++;
	 //if(!saved){blink_led(c); wait_sec(5);}

	 saved_reg = (GPIOA->OTYPER == otype);
	 saved &= saved_reg;
	 c++;
	 //if(!saved_reg){blink_led(c); wait_sec(5);}

	 saved_reg = (GPIOA->PUPDR == pupdr);
	 saved &= saved_reg;
	 c++;
	 //if(!saved_reg){blink_led(c); wait_sec(5);}

	//LED sequence should run only if registers and variables were saved after STOP mode exits
	if(a==5 && b==6 && global_a==5 && global_b ==6 && saved){
		//InitRGBCmd(); //init uart, rgb led driver and services
	}

}

void sleepModeTestWithTimer(void){
	  sequenceHandlerEn = false;
	  FUNCTIONAL_RGB_LED_InitInterruptLongTimer();
	  //start timer interrupt
	  FUNCTIONAL_RGB_LED_StartInterruptTimer();
	  wait_sec(2);
	  sleepModeTest();
}


void standbyModeTest(void){
	initLED();

	/* Enable Power Clock */
	__HAL_RCC_PWR_CLK_ENABLE();

	/* Check and handle if the system was resumed from Standby mode */
	if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET)
	{
	  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

	  /* Infinite loop */
	  while (1)
	  {
		/* Toggle LED2 */
		toggleLED();

		/* Insert a 100ms delay */
		HAL_Delay(100);
	  }
	}


	  /* Allow access to Backup */
	  HAL_PWR_EnableBkUpAccess();

	  /* Reset RTC Domain */
	  __HAL_RCC_BACKUPRESET_FORCE();
	  __HAL_RCC_BACKUPRESET_RELEASE();

	  /* Disable all used wakeup sources: Pin1(PA.0) */
	  HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);

	  /* Clear all related wakeup flags */
	  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

	  /* Re-enable all used wakeup sources: Pin1(PA.0) */
	  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);

	  /*## Enter Standby Mode ####################################################*/
	  /* Request to enter STANDBY mode  */
	  HAL_PWR_EnterSTANDBYMode();

}






int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();


  /* Configure the system clock */
  SystemClock_Config();
  //Init GPIO and I2C1

  MX_I2C1_Init();
  /* Initialize LED*/
  //initLED();

  /* Initialize Button Interrupt */
  //initButtonInterrupt();
  //wait_sec(1);

  //InitRGBCmd(); //init uart, rgb led driver and services
  //stopModeTest();
  //sleepModeTest();
  //sleepModeTestWithTimer();
  //standbyModeTest();
 //TLC59116_init();
  //TLC_RGB_LED_Test();

  InitRGBCmd();
  int count = 0;
  while (1)
  {
	  wait_sec(1);
	  count++;
	  if(count == 5){
		  FUNCTIONAL_RGB_LED_StopService();
		  count = 100;
	  }

  }

}



//Testing: init only red rgb led
void InitRedLed(void){
	RGB_LED_InitConfigs(0);
	RGB_LED_InitPWMConfig(&RgbLedConfigs[0].red);
}

void StartRedLed(void){
	RGB_LED_StartLED(&RgbLedConfigs[0].red);
}

void StopRedLed(void){
	RGB_LED_StopLED(&RgbLedConfigs[0].red);
	RGB_LED_InitConfigs(0);
	RGB_LED_DeInitPWMConfig(&RgbLedConfigs[0].red);
}

//Init rgb led seq and cmdline
void InitRGBCmd(void){
	FUNCTIONAL_RGB_LED_StartService(FUNCTIONAL_RGB_NUM);
	 //Init_USART();
	 rgbHandle.enabled = true;
	 FUNCTIONAL_RGB_LED_LoadSequence(RGBSEQ_CHARGING);
}

//cmdline loop
void CmdLine(void){
	char *rxMsg = (char*)calloc(maxStr, sizeof(char)); //"!02u200h5000?\n";
	char *seqMsg = (char*)calloc(maxStr, sizeof(char));
	int i = 0;
		  HAL_UART_Receive(&UART_Handle, (uint8_t*)rxMsg, 100, 0xFFFF);
		  if(rxMsg[i]=='!'){
			    i++;
			    seqMsg[0] = rxMsg[i];
			    seqNum = rxMsg[i]-'0';
			    i++;
			    seqMsg[1] = rxMsg[i];
			    seqMsg[2] = '\0';
			    seqNum = seqNum*10 + (rxMsg[i]-'0'); //convert char number to actual number
			    									//ex: '04' = 0*10 + 4 = 4
			    									//ex: '14' = 1*10 + 4 = 14
			    int maxChars = 10;
			    int available = 0;
			    i++;
			    int tries = 0;
			    while(rxMsg[i] != '?'){
			    	tries++;
					if(rxMsg[i]=='u'){
						char * nums = (char*)malloc(sizeof(char)*maxChars);
						i++;
						available = 1;
						int pwCnt = getNumStr(rxMsg, nums, i, maxChars);
						upDur = convertStrToMs(nums, pwCnt);
						i += (pwCnt);
						CrtclSeqUpDur = upDur/1000.0;
						if(available){
							HAL_UART_Transmit(&UART_Handle, (uint8_t*)nums, strlen(nums), 0xFFFF);
							available = 0;
						}
					}
					else if(rxMsg[i]=='d'){
						char * nums = (char*)malloc(sizeof(char)*maxChars);
						i++;
						available = 1;
						int pwCnt = getNumStr(rxMsg, nums, i, maxChars);
						downDur = convertStrToMs(nums, pwCnt);
						i += (pwCnt);
						CrtclSeqDwnDur = downDur/1000.0;
						if(available){
							HAL_UART_Transmit(&UART_Handle, (uint8_t*)nums, strlen(nums), 0xFFFF);
							available = 0;
						}
					}
					else if(rxMsg[i]=='h'){
						char * nums = (char*)malloc(sizeof(char)*maxChars);
						i++;
						available = 1;
						int pwCnt = getNumStr(rxMsg, nums, i, maxChars);
						holdDur = convertStrToMs(nums, pwCnt);
						i += (pwCnt);
						CrtclSeqHldDur = holdDur/1000.0;
						if(available){
							HAL_UART_Transmit(&UART_Handle, (uint8_t*)nums, strlen(nums), 0xFFFF);
							available = 0;
						}
					}
					else if(rxMsg[i]=='m'){
						char * nums = (char*)malloc(sizeof(char)*maxChars);
						i++;
						available = 1;
						int pwCnt = getNumStr(rxMsg, nums, i, maxChars);
						modeNum = convertStrToMs(nums, pwCnt);
						i += (pwCnt);
						if(modeNum ==1){
							CrtclSeqMode = RGB_LIN_MODE;
						}
						else if(modeNum == 2){
							CrtclSeqMode = RGB_EXP_MODE;
						}
						if(available){
							HAL_UART_Transmit(&UART_Handle, (uint8_t*)nums, strlen(nums), 0xFFFF);
							available = 0;
						}
					}
					if(tries>= 100){tries = 0; break;}
			   }

			    //rxMsg = (char*)calloc(maxStr, sizeof(char)); //reset
			    if(seqNum < RGB_SEQ_COUNT){
					FUNCTIONAL_RGB_LED_LoadSequence((rgb_seq_type_t) seqNum);
					HAL_UART_Transmit(&UART_Handle, (uint8_t*)seqMsg, strlen(seqMsg), 0xFFFF);
				}
		  }
}

//get number string
int getNumStr(char * buffer, char * nums, int startIndex, int maxChars){
	int pwCnt = 0;
		for(int j = startIndex, i = 0; j<(startIndex + maxChars); j++, i++){
			if(buffer[j] >= '0' && buffer[j] <= '9'){
				nums[i] = buffer[j];
				pwCnt++;
			}
			else{
				nums[i] = '\0';
				break;
			}
		}
		return pwCnt;
}

//convert chars to millisec num
int convertStrToMs(char * numStr, int powerCount){
	int total = 0;
	int count = powerCount - 1;
	if(count <= 0){return numStr[0] - '0';}

	for(int i = 0; i< count; i++){
		int num = numStr[i] - '0';
		num = num * pow(10,count-i); //ex: if numStr = '50' then count = 2 - 1 = 1, 5*10^1 + 0*10^0 = 50
		total += num;
	}
	return total;
}


/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_13){
		//GPIOA->ODR ^= GPIO_PIN_5;
	}

}

void HAL_UART_MspInit(UART_HandleTypeDef *husart){
	// sort out clocks
	if(husart->Instance == USART2){
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_USART2_CLK_ENABLE();

		/* Configure USART2 Tx (PA.02) and RX (PA.03) as alternate function push-pull */
		GPIO_InitTypeDef  GPIO_TX_InitStructure;
		GPIO_TX_InitStructure.Pin = GPIO_PIN_2;
		GPIO_TX_InitStructure.Speed = GPIO_SPEED_LOW;
		GPIO_TX_InitStructure.Mode = GPIO_MODE_AF_PP;
		GPIO_TX_InitStructure.Pull = GPIO_PULLUP;
		GPIO_TX_InitStructure.Alternate = GPIO_AF7_USART2;
		HAL_GPIO_Init(GPIOA, &GPIO_TX_InitStructure);// Map USART2 TX to A.02

		GPIO_InitTypeDef  GPIO_RX_InitStructure;
		GPIO_RX_InitStructure.Pin = GPIO_PIN_3;
		GPIO_RX_InitStructure.Speed = GPIO_SPEED_LOW;
		GPIO_RX_InitStructure.Mode = GPIO_MODE_AF_PP;
		GPIO_RX_InitStructure.Pull = GPIO_PULLUP;
		GPIO_RX_InitStructure.Alternate = GPIO_AF7_USART2;
		HAL_GPIO_Init(GPIOA, &GPIO_RX_InitStructure);// Map USART2 RX to A.02
	}

	if(husart->Instance == USART1){
			__HAL_RCC_GPIOA_CLK_ENABLE();
			__HAL_RCC_USART1_CLK_ENABLE();

			/* Configure USART2 Tx (PA.010) and RX (PA.09) as alternate function push-pull */
			GPIO_InitTypeDef  GPIO_TX_InitStructure;
			GPIO_TX_InitStructure.Pin = GPIO_PIN_9;
			GPIO_TX_InitStructure.Speed = GPIO_SPEED_LOW;
			GPIO_TX_InitStructure.Mode = GPIO_MODE_AF_PP;
			GPIO_TX_InitStructure.Pull = GPIO_PULLUP;
			GPIO_TX_InitStructure.Alternate = GPIO_AF7_USART1;
			HAL_GPIO_Init(GPIOA, &GPIO_TX_InitStructure);// Map USART2 TX to A.09

			GPIO_InitTypeDef  GPIO_RX_InitStructure;
			GPIO_RX_InitStructure.Pin = GPIO_PIN_10;
			GPIO_RX_InitStructure.Speed = GPIO_SPEED_LOW;
			GPIO_RX_InitStructure.Mode = GPIO_MODE_AF_PP;
			GPIO_RX_InitStructure.Pull = GPIO_PULLUP;
			GPIO_RX_InitStructure.Alternate = GPIO_AF7_USART1;
			HAL_GPIO_Init(GPIOA, &GPIO_RX_InitStructure);// Map USART2 RX to A.010
		}
}

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
