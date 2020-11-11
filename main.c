/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
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
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	uint16_t raw;
	char msg[20];
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	LCD_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//	uint8_t x = 0;
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		POINT_COLOR = BLACK;
		LCD_DrawRectangle(20, 20, 220, 300);
		LCD_Fill(21, 21, 219, 299, CYAN);

		LCD_ShowString(30, 21, 190, 16, 16, (uint8_t*) "TFTLCD TEST1");
		LCD_ShowString(21, 40, 190, 16, 16, (uint8_t*) "TFTLCD TEST2");
		LCD_ShowNum(160,60,1234,4,15);

//		if (false) {
//			HAL_ADC_Start(&hadc1); // Wait for regular group conversion to be completed
//			HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); // Get ADC value
//			raw = HAL_ADC_GetValue(&hadc1); // the voltage should be raw * (3.3/4096)(12bits)
//			// Convert to string and print
//			double v = raw * (3.3 / 4096);
//			double t = ((1.43 - v) / 4.3) + 25;
//			sprintf(msg, "%d\r\n", t);
//			HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen(msg),HAL_MAX_DELAY);
//		}
//		switch (x) {
//		case 0:
//			LCD_Clear(WHITE);
//			BACK_COLOR = WHITE;
//			break;
//		case 1:
//			LCD_Clear(BLACK);
//			BACK_COLOR = BLACK;
//			break;
//		case 2:
//			LCD_Clear(BLUE);
//			BACK_COLOR = BLUE;
//			break;
//		case 3:
//			LCD_Clear(RED);
//			BACK_COLOR = RED;
//			break;
//		case 4:
//			LCD_Clear(MAGENTA);
//			BACK_COLOR = MAGENTA;
//			break;
//		case 5:
//			LCD_Clear(GREEN);
//			BACK_COLOR = GREEN;
//			break;
//		case 6:
//			LCD_Clear(CYAN);
//			BACK_COLOR = CYAN;
//			break;
//		case 7:
//			LCD_Clear(YELLOW);
//			BACK_COLOR = YELLOW;
//			break;
//		case 8:
//			LCD_Clear(BRRED);
//			BACK_COLOR = BRRED;
//			break;
//		case 9:
//			LCD_Clear(GRAY);
//			BACK_COLOR = GRAY;
//			break;
//		case 10:
//			LCD_Clear(LGRAY);
//			BACK_COLOR = LGRAY;
//			break;
//		case 11:
//			LCD_Clear(BROWN);
//			BACK_COLOR = BROWN;
//			break;
//		}
//		POINT_COLOR = RED;
//		LCD_ShowString(30, 40, 200, 24, 24, (uint8_t*) "Mini STM32 ^_^");
//		LCD_ShowString(30, 70, 200, 16, 16, (uint8_t*) "TFTLCD TEST");
//		POINT_COLOR = BLACK;
//		LCD_DrawRectangle(30, 150, 210, 190);
//		LCD_Fill(31, 151, 209, 189, YELLOW);
//		x++;
//		if (x == 12)
//			x = 0;
//		HAL_Delay(2000);
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
