
#include "main.h"
#include "lcd.h"
#include "string.h"

UART_HandleTypeDef huart1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

//-------------------------------------------------------------------------------------------------

const int left=21;
const int right=219;
const int top=21;
const int bottom=299;
const int lineHeight=20;
const int width=190;
const int fontSize=16;
uint8_t rxBuffer[20];

//int jiange=20;
//int max_y=300-16-20;
//int count=0;
//static unsigned char uRx_Data[14][1024] = { 0 };
//static unsigned char uLength=0;
//int start[14];

int lineCount=0;
void showString(char msg[], int position, int color) {
	POINT_COLOR=color;
	if (position==right) {
		LCD_ShowString(right-(strlen(msg))*(fontSize/2), top+lineHeight+lineCount*lineHeight, width, fontSize, fontSize, (uint8_t*) msg);
	}
	else if (position==left) {
		LCD_ShowString(left, top+lineHeight+lineCount*lineHeight, width, fontSize, fontSize, (uint8_t*) msg);
	}
	else {
		return;
	}
	lineCount++;
	POINT_COLOR=BLACK;
}

void showInput(uint8_t* msg, int length, int position, int color) {
	POINT_COLOR=color;
	if (position==right) {
		LCD_ShowString(right-(length-1)*(fontSize/2), top+lineHeight+lineCount*lineHeight, width, fontSize, fontSize, msg);
	}
	else if (position==left) {
		LCD_ShowString(left, top+lineHeight+lineCount*lineHeight, width, fontSize, fontSize, msg);
	}
	else {
		return;
	}
	lineCount++;
	POINT_COLOR=BLACK;
}

void showNumber() {
	return;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		static unsigned char uRx_Data[1024] = { 0 };
		static unsigned char uLength = 0;
		if (rxBuffer[0] == '\n') {
			HAL_UART_Transmit(&huart1, uRx_Data, uLength, 0xffff);
			showInput((uint8_t*) uRx_Data, uLength, right, BLACK);
			uLength = 0;
		} else {
			uRx_Data[uLength] = rxBuffer[0];
			uLength++;
		}
	}
}

int main(void) {
	HAL_Init();

	SystemClock_Config();

	LCD_Init();

	MX_GPIO_Init();
	MX_USART1_UART_Init();

	HAL_UART_Receive_IT(&huart1, (uint8_t*) rxBuffer, 1);

//-------------------------------------------------------------------------------------------------
	LCD_Clear(CYAN);
	POINT_COLOR=BLACK;
	LCD_DrawRectangle(20, 20, 220, 300);
	LCD_Fill(left, top, right, bottom, WHITE);

	showString("Question", right, BLUE);
	showString("fffffffffffffffff", right, BLACK);
	showString("Answer", left, BLUE);
	showString("fuckyou", left, BLACK);
	showString("Question", right, BLUE);
//-------------------------------------------------------------------------------------------------

	while (1) {
//		LCD_ShowString(21, 21, 190, 16, 16,
//				(uint8_t*) "TFTLCDddfafdadfadsTESsdfsdddT1");
//		LCD_ShowString(21, 41, 190, 16, 16, (uint8_t*) "TFTLCD TEST2");
//		POINT_COLOR = RED;
//		int now_y=21;
//		for (int i=0 ; i<14;i++){
//			LCD_ShowString(start[i], now_y, 200, 16, 16, (uint8_t*) uRx_Data[i]);
//			if (now_y<=max_y)
//				now_y+=jiange;
//		}
//		LCD_ShowNum(160, 60, 1234, 4, 15);

//		HAL_Delay(2000);
	}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

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
	if (HAL_UART_Init(&huart1) != HAL_OK) {
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
static void MX_GPIO_Init(void) {

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
void Error_Handler(void) {
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
