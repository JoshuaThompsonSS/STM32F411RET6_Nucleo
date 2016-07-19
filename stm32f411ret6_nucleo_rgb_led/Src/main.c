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
#include "stm32f4xx_hal_uart.h"
#include <math.h>
#include "stdlib.h"
#include "string.h"
#include "StmFw_Upgrade.h"



/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
#define maxStr 100

/* Private variables -----------------------------------------------------*/
GPIO_InitTypeDef  GPIO_InitStruct;

UART_InitTypeDef UART_InitStructure;
UART_InitTypeDef UART1_InitStructure;
UART_HandleTypeDef UART_Handle;
UART_HandleTypeDef UART1_Handle;


/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


/* Function prototypes -----------------------------------------------*/
void uart1_putc(uint8_t c){
	uint8_t d[1];
	d[0] = c;
	HAL_UART_Transmit(&UART1_Handle, d, 1, 0xFFFF);
}

uint8_t uart1_getc(void){
	uint8_t d[1];
	HAL_UART_Receive(&UART1_Handle, (uint8_t*)d, 1, 0xFFFF);
	return d[0];
}

void uart1_gets(unsigned char * buff, int len){
	HAL_UART_Receive(&UART1_Handle, (uint8_t*)buff, len, 0xFFFF);
}

void uart2_putc(uint8_t c){
	uint8_t d[1];
	d[0] = c;
	HAL_UART_Transmit(&UART_Handle, d, 1, 0xFFFF);
}

uint8_t uart2_getc(void){
	uint8_t d[1];
	HAL_UART_Receive(&UART_Handle, d, 1, 0xFFFF);
	return d[0];
}

void serial_write(const char * buff, int len){
  for(int i = 0; i<len; i++){
	  	  uart1_putc(buff[i]);
      }
}

unsigned char serial_read(void){
  //uint8_t d = uart1_getc();

  return uart1_getc();
}

void serial_reads(unsigned char* buff, int len){
  /*
  for(int i = 0; i<len; i++){
      buff[i] = uart1_getc();
      }
  */
  uart1_gets(buff,len);
}

void host_write(const char * buff){
	return;
	int len = strlen(buff); //buff must be null terminated
	for(int i = 0; i<len; i++){
		  	  uart2_putc(buff[i]);
	}
}

void host_read(char * data, int len){
  for(int i = 0; i<len; i++){
      data[i] = uart2_getc();
      }
}

void delay(uint32_t ms){
	HAL_Delay(ms);
}

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
	UART1_InitStructure.Parity = UART_PARITY_NONE;//UART_PARITY_EVEN;
	UART1_InitStructure.Mode = UART_MODE_TX_RX;
	UART1_InitStructure.HwFlowCtl = UART_HWCONTROL_NONE;
	/* Configure USART */
	UART1_Handle.Instance = USART1;
	UART1_Handle.Init = UART1_InitStructure;
	/* Init and Enable the USART */
	HAL_UART_Init(&UART1_Handle);

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



//wait function
void wait_sec(float sec){
	HAL_Delay(sec*1000);
}

//toggle led
void toggleLED(void){
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}

void setup() {
  // Open serial communications and wait for port to open:
  Init_USART(); //USART2
  Init_USART1(); //USART1 PA9 (TX) and PA10 (RX)
  uart_handlers_t uartPtrs = {&serial_read, &serial_reads, &serial_write, &host_write, &delay, &host_read};

  load_uart_handlers(&uartPtrs);

}


int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();


  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize LED*/
  initLED();

  setup();
  wait_sec(1);
  /*
  char *rxMsg = (char*)calloc(maxStr, sizeof(char)); //"!02u200h5000?\n";
  uint8_t nums[] = {0x7F};
  HAL_UART_Transmit(&UART1_Handle, (uint8_t*)nums, 1, 0xFFFF);
  wait_sec(0.1);
  HAL_UART_Receive(&UART1_Handle, (uint8_t*)rxMsg, 1, 0xFFFF);
  if(rxMsg[0] == 0x79){
	 toggleLED();
  }
  */
  wait_sec(5);
  host_write("Getting init chip\n");
  initChip();
  wait_sec(1);
  /*
  int ver = cmdGet();
  if(ver==0x31){
	  host_write("Get cmd ok\n");
  }
  wait_sec(1);
  */
  //int ok = cmdGetVersion();//cmdGeneric(0x00);
  wait_sec(1);
  if(0){
	  //host_write("cmd get send ok\n");
	  //int len = uart1_getc();
	  //int ver = uart1_getc();
	  host_write("got ver and lend");

  }
  else{
	  host_write("cmd get send failed\n");
  }

  while (1)
  {

	 wait_sec(2);
	 toggleLED();
	 //host_write("Starting init chip\n");
	 mdebug(0, "Starting init chip\n");
	 /*
	 char *rxMsg = (char*)calloc(maxStr, sizeof(char)); //"!02u200h5000?\n";
	 HAL_UART_Receive(&UART_Handle, (uint8_t*)rxMsg, 1, 0xFFFF);
	 if(rxMsg[0]=='J'){
		 toggleLED();
	 }
	 */


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
