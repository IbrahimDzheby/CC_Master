/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t timer_tick_flag = 0;

uint16_t value12;
uint16_t adc_value = 0;
uint16_t V = 0;
uint16_t adc12 = 0;
uint8_t bit_index = 0;
uint8_t transfer_done = 0;
uint8_t timer_flag = 0;
uint8_t sck_state = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

#define ADC0_CS_LOW()  HAL_GPIO_WritePin(GPIOC, A0_Pin, GPIO_PIN_RESET)
#define ADC0_CS_HIGH() HAL_GPIO_WritePin(GPIOC, A0_Pin, GPIO_PIN_SET)
#define ADC1_CS_LOW()  HAL_GPIO_WritePin(GPIOC, A0_Pin, GPIO_PIN_SET)
#define ADC1_CS_HIGH() HAL_GPIO_WritePin(GPIOC, A0_Pin, GPIO_PIN_RESET)
//#define ADC_CS_LOW()  HAL_GPIO_WritePin(GPIOC, E_Pin, GPIO_PIN_RESET)
//#define ADC_CS_HIGH() HAL_GPIO_WritePin(GPIOC, E_Pin, GPIO_PIN_SET)
#define SCK_HIGH()    HAL_GPIO_WritePin(GPIOA, SCK_Pin, GPIO_PIN_SET)
#define SCK_LOW()     HAL_GPIO_WritePin(GPIOA, SCK_Pin, GPIO_PIN_RESET)
#define MISO_READ()   HAL_GPIO_ReadPin(GPIOA, MISO_Pin)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
uint16_t ADC1_BitBang_Read(void)
{
    uint16_t value = 0;

    ADC1_CS_LOW(); // опускаем CS
    HAL_Delay(1); // короткая пауза (~1 мкс, можно уменьшить)

    for(int i = 0; i < 16; i++)
    {
        SCK_HIGH();            // подняли такт
        HAL_Delay(1);          // держим немного
        value <<= 1;           // сдвигаем
        if(MISO_READ())        // читаем бит
            value |= 1;
        SCK_LOW();             // опустили такт
        HAL_Delay(1);          // удерживаем низкий уровень
    }

    ADC1_CS_HIGH(); // подняли CS
    return value & 0x0FFF; // оставляем 12 бит
}*/

void WaitTimerTick(void)
{
    while (!timer_tick_flag);  // ждём, пока не установится
    timer_tick_flag = 0;       // сбрасываем после ожидания
}

void ADC1_Timer_ReadStart(void)
{
    bit_index = 0;
    adc_value = 0;
    transfer_done = 0;

    ADC1_CS_LOW();             // опускаем CS
    HAL_TIM_Base_Start_IT(&htim2); // запускаем таймер
}

void ADC_ChipSelect(uint8_t adc_index)
{
	uint8_t bit0 = (adc_index >> 0) & 0x01;
	uint8_t bit1 = (adc_index >> 1) & 0x01;
	uint8_t bit2 = (adc_index >> 2) & 0x01;
	uint8_t bit3 = (adc_index >> 3) & 0x01;

	HAL_GPIO_WritePin(GPIOC, A0_Pin, bit0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, A1_Pin, bit1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, A2_Pin, bit2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, A3_Pin, bit3 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void ADC_LevelSet(uint8_t adc_index, GPIO_PinState adc_level)
{
	uint8_t board_index;
	board_index = adc_index / 14;

	switch (board_index)
	{
		case (0):
			HAL_GPIO_WritePin(GPIOC, E1_Pin, adc_level);
			break;
		case (1):
			HAL_GPIO_WritePin(GPIOC, E2_Pin, adc_level);
			break;
		case (2):
			HAL_GPIO_WritePin(GPIOC, E3_Pin, adc_level);
			break;
		case (3):
			HAL_GPIO_WritePin(GPIOC, E4_Pin, adc_level);
			break;
		case (4):
			HAL_GPIO_WritePin(GPIOC, E5_Pin, adc_level);
			break;
		default:
			HAL_GPIO_WritePin(GPIOC, E1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, E2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, E3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, E4_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, E5_Pin, GPIO_PIN_SET);
			break;
	}

}

void ADC_On(uint8_t adc_index)
{
	HAL_TIM_Base_Start_IT(&htim2); // запускаем таймер

	ADC_LevelSet(adc_index, GPIO_PIN_RESET);
	for(int i=0; i<22; i++)
	{
		WaitTimerTick();
	}
	ADC_LevelSet(adc_index, GPIO_PIN_SET);

	HAL_TIM_Base_Stop_IT(&htim2);
}

void ADC_DataRead(uint8_t adc_index)
{
	HAL_TIM_Base_Start_IT(&htim2); // запускаем таймер

	ADC_LevelSet(adc_index, GPIO_PIN_RESET);

	for(int i=0; i<16; i++)
	{
	    // фронт
	    SCK_HIGH();
	    WaitTimerTick();
	    adc_value <<= 1;
	    if(MISO_READ()) adc_value |= 1;

	    // спад
	    SCK_LOW();
	    WaitTimerTick();
	}

	ADC_LevelSet(adc_index, GPIO_PIN_SET);
	HAL_TIM_Base_Stop_IT(&htim2);

	adc12 = adc_value & 0x0FFF; // 12 бит ADC
	V = (uint16_t)(adc12 * 1000 * 3.3f * 6.6f / 4095.0f);
	printf("ADC = %u, V = %u mV\n\r", adc12, V);
	sck_state = 0;
	bit_index = 0;
	return;
}

void ADC_Off(uint8_t adc_index)
{
	HAL_TIM_Base_Start_IT(&htim2); // запускаем таймер

	ADC_LevelSet(adc_index, GPIO_PIN_RESET);
	for(int i=0; i<8; i++)
	{
		WaitTimerTick();
	}
	ADC_LevelSet(adc_index, GPIO_PIN_SET);

	HAL_TIM_Base_Stop_IT(&htim2);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  for (uint8_t adc_index = 0; adc_index <= 1; adc_index++)
	  {
			bit_index = 0;
			adc_value = 0;
			ADC_ChipSelect(adc_index);
			ADC_On(adc_index);
			HAL_Delay(1);
			ADC_DataRead(adc_index);
			HAL_Delay(1);
			ADC_Off(adc_index);
	  }
	  HAL_Delay(100);
	  printf("\033[2J");  // очистить весь экран
	  printf("\033[H");   // курсор в начало
	  /*
		bit_index = 0;
		adc_value = 0;
		uint8_t adc_index = 1;
		ADC_ChipSelect(adc_index);
		ADC_On(adc_index);
		HAL_Delay(1);
		ADC_DataRead(adc_index);
		HAL_Delay(1);
		ADC_Off(adc_index);
		HAL_Delay(100);*/

/*
	  ADC1_Timer_ReadStart();
	  while(!transfer_done)
	  {

	  } // ждём окончания
	  uint16_t adc12 = adc_value & 0x0FFF; // 12 бит ADC
	  uint16_t V = (uint16_t)(adc12 * 1000 * 3.3f * 6.6f / 4095.0f);
	  printf("\rADC = %u, V = %u mV", adc12, V);
	  HAL_Delay(100);*/



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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3599;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, E3_Pin|E4_Pin|USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, A3_Pin|A2_Pin|A1_Pin|A0_Pin
                          |E1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, E2_Pin|E5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SCK_Pin */
  GPIO_InitStruct.Pin = SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MISO_Pin */
  GPIO_InitStruct.Pin = MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MISO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : E3_Pin E4_Pin USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = E3_Pin|E4_Pin|USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : A3_Pin A2_Pin A1_Pin E1_Pin */
  GPIO_InitStruct.Pin = A3_Pin|A2_Pin|A1_Pin|E1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : A0_Pin */
  GPIO_InitStruct.Pin = A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(A0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : E2_Pin E5_Pin */
  GPIO_InitStruct.Pin = E2_Pin|E5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2)
    {
    	timer_tick_flag = 1;

    	/*if(bit_index >= 16)  // закончили чтение
	{
	  ADC1_CS_HIGH();   // поднимаем CS
	  transfer_done = 1;
	  HAL_TIM_Base_Stop_IT(&htim2);
	  sck_state = 0;
	  return;
	}

	if(sck_state == 0)
	{
	  SCK_HIGH();                  // фронт SCK
	  adc_value <<= 1;
	  if(MISO_READ()) adc_value |= 1;
	  sck_state = 1;
	}
	else
	{
	  SCK_LOW();                   // спад SCK
	  sck_state = 0;
	  bit_index++;
	}*/
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
