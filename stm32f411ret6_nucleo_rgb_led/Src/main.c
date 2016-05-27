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
#include "rgb_led.h"


/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef TimHandle;
TIM_OC_InitTypeDef PwmConfig;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */


/* Function prototypes -----------------------------------------------*/


//Initialize the board debug led as output
void initLED(void){
	//enable GPIOA peripheral clock
	__HAL_RCC_GPIOA_CLK_ENABLE();
	//enable GPIOA pin 5 as output
	GPIO_InitTypeDef  GPIO_InitStruct;
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
void wait_sec(int sec){
	HAL_Delay(sec*1000);
}

//toggle led
void toggleLED(void){
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}


int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize LED*/
  //initLED();

  /* Initialize Button Interrupt */
  //initButtonInterrupt();

  /* Initialize PWM on TIM2 Channel 2 */
  //initPwmTimer();
  RGB_LED_InitLEDConfig(&RGB1_RedTimHandle, &RGB1_RedPwmConfig, RGB1_RED_TIM_REG, RGB1_RED_TIM_CH);
  RGB_LED_StartLED(&RGB1_RedTimHandle, RGB1_RED_TIM_CH);
  uint32_t duty_cycle_percent = 0;

  while (1)
  {
	  //Interrupt will toggle led (GPIOA 5) when button on GPIOC 13 is pressed
	  //Interrupt handler is EXTI15_10_IRQHandler (defined in stm32f4xx_it.c) and calls the led toggle code
	  //when the interrupt is generated on GPIOC pin 13 (button)

	  //continuously dim and then brighten LED
	  wait_sec(1); //update duty cycle until 100% then restart
	  RGB_LED_UpdateDutyCycle(duty_cycle_percent, &RGB1_RedTimHandle, &RGB1_RedPwmConfig);
	  duty_cycle_percent += 10;
	  if(duty_cycle_percent >= 100){duty_cycle_percent = 0;} //restart after reaching 100% brightness


  }

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
		GPIOA->ODR ^= GPIO_PIN_5;
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
