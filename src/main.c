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
#include "mxconstants.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void HF_TIM_Init(TIM_HandleTypeDef *htim_HF);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
TIM_HandleTypeDef    TimHandleHF;
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //MX_TIM3_Init();
  //MX_TIM4_Init();

  HF_TIM_Init(&TimHandleHF);

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
  /*
  // Enable GPIOB & TIM4 clocks
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_TIM4_CLK_ENABLE();

  // Set up GPIOB1 (timer 3, channel 4) in alternate function (AF) mode
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Pin   = GPIO_PIN_6;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Set up the counter itself.
  TIM_Base_InitTypeDef TIM_BaseStruct;
  TIM_HandleTypeDef TIM_HandleStruct;
  TIM_HandleStruct.Instance = TIM4;
  // No prescaler (PSC), so the count (CNT) will count up 64 million times per second
  TIM_BaseStruct.Prescaler   = 0;//(uint32_t) ((SystemCoreClock / 36000) - 1);
  TIM_BaseStruct.CounterMode = TIM_COUNTERMODE_UP;

  // When the counter hits the Period value, it will reset back to 0.
  // This is the ARR register. In this case, the clock is 64mhz so:
  //   64,000,000 / 842 / 2 = 38,005hz
  // The divide by two is because it takes two toggles to create one wave
  TIM_BaseStruct.Period = 1000;
  TIM_BaseStruct.ClockDivision = 0;
  TIM_HandleStruct.Init    = TIM_BaseStruct;
  TIM_HandleStruct.Channel = HAL_TIM_ACTIVE_CHANNEL_1;


  // Initialize the timer hardware in output compare mode
  HAL_TIM_OC_Init(&TIM_HandleStruct);

  // Set the parameters for output compare
  TIM_OC_InitTypeDef TIM_OCStruct;

  // Toggle the associated pin when CNT >= CCR
  TIM_OCStruct.OCMode = TIM_OCMODE_TOGGLE;

  // This is the counter value when the the channel will be toggled
  // For this simple case, the value here does not matter.
  TIM_OCStruct.Pulse = 0;

  // Configure the channel.
  HAL_TIM_OC_ConfigChannel(&TIM_HandleStruct, &TIM_OCStruct, TIM_CHANNEL_1);

  // Start the timer comparing
  HAL_TIM_OC_Start(&TIM_HandleStruct, TIM_CHANNEL_1);
  */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
  /* USER CODE END WHILE */
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	  HAL_Delay(200);
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	  HAL_Delay(200);
  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
     PA2   ------> USART2_TX
     PA3   ------> USART2_RX
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */
void HF_TIM_Init(TIM_HandleTypeDef *htim_HF){

	GPIO_InitTypeDef GPIO_HF_TIM;
	TIM_OC_InitTypeDef TIM_HF_CH;

	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_TIM4_CLK_ENABLE();


	GPIO_HF_TIM.Pin = GPIO_PIN_6;
	GPIO_HF_TIM.Mode = GPIO_MODE_AF_PP;
	GPIO_HF_TIM.Pull = GPIO_NOPULL;
	GPIO_HF_TIM.Speed = GPIO_SPEED_HIGH;
	GPIO_HF_TIM.Alternate = GPIO_AF2_TIM4;

	HAL_GPIO_Init(GPIOB, &GPIO_HF_TIM);


	HAL_TIM_OC_DeInit(htim_HF);
	/* PWM_frequency = timer_tick_frequency / (TIM_Period + 1) */
	htim_HF->Instance = TIM4;
	htim_HF->Init.Period = ((uint32_t)2333);
	htim_HF->Init.Prescaler = 2;
	htim_HF->Init.CounterMode = TIM_COUNTERMODE_UP;
	htim_HF->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

	HAL_TIM_Base_Init(htim_HF);

	HAL_TIM_OC_Init(htim_HF);

	/* PWM mode 2 = Clear on compare match */
	/* PWM mode 1 = Set on compare match */
	TIM_HF_CH.OCMode = TIM_OCMODE_PWM1;
	/* To get proper duty cycle, you have simple equation */
	/* pulse_length = ((TIM_Period + 1) * DutyCycle) / 100 - 1 */
	/* where DutyCycle is in percent, between 0 and 100% */
	TIM_HF_CH.Pulse = ((uint32_t)2333)/4;
	TIM_HF_CH.OCPolarity = TIM_OCPOLARITY_HIGH;
	TIM_HF_CH.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	TIM_HF_CH.OCFastMode = TIM_OCFAST_DISABLE;
	TIM_HF_CH.OCIdleState = TIM_OCIDLESTATE_RESET;
	TIM_HF_CH.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	HAL_TIM_OC_ConfigChannel(htim_HF, &TIM_HF_CH, TIM_CHANNEL_1);

	TIM_SET_CAPTUREPOLARITY(htim_HF, TIM_CHANNEL_1, TIM_CCxN_ENABLE | TIM_CCx_ENABLE );

	HAL_TIM_OC_Start(htim_HF, TIM_CHANNEL_1);

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler */
}

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
