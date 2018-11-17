
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "main.h"
#include "stm32f0xx_hal.h"
#include "rtc.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#define TLC5925_WIDTH 16
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint16_t* active_leds_columns;
uint8_t cur_column = 0;
uint8_t mutex = 0;
GPIO_TypeDef* COL_port_array[] = { LCOL1_GPIO_Port, LCOL2_GPIO_Port,
LCOL3_GPIO_Port, LCOL4_GPIO_Port, LCOL5_GPIO_Port, LCOL6_GPIO_Port,
LCOL7_GPIO_Port, LCOL8_GPIO_Port, LCOL9_GPIO_Port, LCOL10_GPIO_Port, NULL };
uint16_t COL_pin_array[] = { LCOL1_Pin, LCOL2_Pin, LCOL3_Pin, LCOL4_Pin,
LCOL5_Pin, LCOL6_Pin, LCOL7_Pin, LCOL8_Pin, LCOL9_Pin, LCOL10_Pin, 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
	active_leds_columns = (uint16_t*) malloc(sizeof(uint16_t) * TLC5925_WIDTH);
	if (active_leds_columns == NULL)
		Error_Handler();
	memset(active_leds_columns, 0, sizeof(uint16_t) * TLC5925_WIDTH);
	HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, GPIO_PIN_RESET);
	HAL_TIM_Base_Start_IT(&htim7);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	}
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure LSE Drive Capability 
    */
  HAL_PWR_EnableBkUpAccess();

  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_HIGH);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL5;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/**
 * Delay function using Timer6 as a timebase. Delays for the specified amount of us.
 */
void __delay_us(uint16_t us) {
	htim6.Instance->CR1 |= TIM_CR1_CEN; //Run Timer
	while (htim6.Instance->CNT < us)
		; //Wait and be vewwy vewwy quiet
	htim6.Instance->CR1 &= ~(TIM_CR1_CEN); //Stop Timer
	htim6.Instance->CNT = 0; //Reset Value
}

/**
 * Function that puts the 16bit int specified in out into the TLC5925 LED Driver on the board.
 * LE is pulsed so the buffers are cleared. LE and CLK Signals are cleared at return. SDI is not
 */
void TLC_Out(uint16_t out) {

//Sanity check. We care about these two lines. SDI is irrelevant and OE is controlled elsewhere
	HAL_GPIO_WritePin(LE_GPIO_Port, LE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_RESET);

//Shift through the bits one by one and put em in the chip. Right now 1us delay bc debugging
	for (int i = 0; i < 16; i++) {
		HAL_GPIO_WritePin(SDI_GPIO_Port, SDI_Pin, (out << i) & 0x01);
		__delay_us(1);
		HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_SET);
		__delay_us(1);
		HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_RESET);
	}

//Pulse LE to put the buffers through to the actual pins. Also 1us bc debugging. sue me
	HAL_GPIO_WritePin(LE_GPIO_Port, LE_Pin, GPIO_PIN_SET);
	__delay_us(1);
	HAL_GPIO_WritePin(LE_GPIO_Port, LE_Pin, GPIO_PIN_RESET);

}

void ColumnOut(GPIO_TypeDef* port, uint16_t pin) {

	HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
	while (mutex)
		;
	mutex = ~mutex;
	TLC_Out(active_leds_columns[cur_column]);
	__delay_us(5);
	cur_column = (cur_column + 1) % 10;
	mutex = ~mutex;
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);

}

/**
 * Specified for the german front panel.
 */
void DecodeTime(RTC_TimeTypeDef time) {

	//Wait for mutex and lock it if you can
	while (mutex)
		;
	mutex = ~mutex;

	//Reset
	memset(active_leds_columns, 0, sizeof(uint16_t) * TLC5925_WIDTH);

	//Always needed
	active_leds_columns[0] |= 0xDC00; //0xDC00 = [Es ist] on
	active_leds_columns[8] |= 0x0007; //0x0007 = [Uhr] on

	uint8_t min_index = time.Minutes / 5;
	uint8_t hour_index = time.Hours % 12;

	/**	Minute Math
	 * 0-59 / 5 = 0-11
	 * 0 = - [-]
	 * 1 =  [Fünf + Nach]x
	 * 2 =  [Zehn + Nach]x
	 * 3 =  [Viertel + Nach]x
	 * 4 =  [Zwanzig + Nach]x
	 * 5 =  [Fünf + Vor + Halb]
	 * 6 =  [Halb] 					+h
	 * 7 =  [Fünf + Nach + Halb]	+h
	 * 8 =  [Zwanzig + Vor]			+h
	 * 9 =  [Drei + viertel]		+h
	 * 10 = [Zehn + Vor]			+h
	 * 11 = [Fünf + Vor]			+h
	 *
	 * [Nach] 	 = 		Row: 3 | Leds: 0x1E00
	 * [Vor]  	 =		Row: 3 | Leds: 0xE000
	 * [Fünf] 	 =		Row: 0 | Leds: 0x00F0
	 * [Zehn] 	 =		Row: 1 | Leds: 0xF000
	 * [Viertel] =		Row: 2 | Leds: 0x07F0
	 * [Zwanzig] =		Row: 1 | Leds: 0x07F0
	 * [Drei]	 =		Row: 2 | Leds: 0x7800
	 * [Halb]	 =		Row: 3 | Leds: 0x00F0
	 */
	if (min_index) {
		if (min_index < 5 || min_index == 7) {	//[Nach]
			active_leds_columns[3] |= 0x1E00;
		}
		if (min_index > 9 || min_index == 8 || min_index == 5) {	//[Vor]
			active_leds_columns[3] |= 0xE000;
		}
		if (min_index % 6 == 1 || min_index % 6 == 5) {	//[Fünf]
			active_leds_columns[0] |= 0x00F0;
		}
		if (min_index % 8 == 2) {	//[Zehn]
			active_leds_columns[1] |= 0xF000;
		}
		if (min_index % 6 == 3) {	//[Viertel]
			active_leds_columns[2] |= 0x07F0;
		}
		if (min_index % 4 == 0) {	//[Zwanzig]
			active_leds_columns[1] |= 0x07F0;
		}
		if (min_index == 9) {	//[Drei]
			active_leds_columns[2] |= 0x7800;
		}
		if (min_index > 4 && min_index < 8) { //[Halb]
			active_leds_columns[3] |= 0x00F0;
		}
		if (min_index > 6) { //+h
			hour_index = (hour_index + 1) % 12;
		}
	}

	//Hour Math
	//F800 = [Zwölf]  on =>  Row: 4 | Leds: 0xF800;
	//F000 = [Eins]   on =>  Row: 5 | Leds: 0xF000;
	//03C0 = [Zwei]   on =>  Row: 6 | Leds: 0x03C0;
	//0F00 = [Drei]   on =>  Row: 8 | Leds: 0x0F00;
	//0F00 = [Vier]   on =>  Row: 5 | Leds: 0x0F00;
	//7800 = [Fünf]   on =>  Row: 7 | Leds: 0x7800;
	//F800 = [Sechs]  on =>  Row: 6 | Leds: 0xF800;
	//03F0 = [Sieben] on =>  Row: 4 | Leds: 0x03F0;
	//00F0 = [Acht]   on =>  Row: 5 | Leds: 0x00F0;
	//F000 = [Neun]   on =>  Row: 8 | Leds: 0xF000;
	//00F0 = [Zehn]   on =>  Row: 7 | Leds: 0x00F0;
	//0700 = [Elf]    on =>  Row: 7 | Leds: 0x0700;
	uint16_t hours_leds[] = { 0xF800, 0xF000, 0x03C0, 0x0F00, 0x0F00, 0x7800,
			0xF800, 0x03F0, 0x00F0, 0xF000, 0x00F0, 0x0700 };
	uint8_t hours_cols[] = { 4, 5, 6, 8, 5, 7, 6, 4, 5, 8, 7, 7 };
	active_leds_columns[hours_cols[hour_index]] |= hours_leds[hour_index];

	//Unlock mutex
	mutex = ~mutex;

}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) {
	RTC_TimeTypeDef time;
	HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
	DecodeTime(time);
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	if (htim->Instance == TIM7) {
		ColumnOut(COL_port_array[cur_column], COL_pin_array[cur_column]);
	}
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
