/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "stdio.h"
#include "usbd_hid.h"

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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define MOUSE_REPORT_SIZE 5
#define MOVE_DELAY 18
#define MOVE_DELTA 10
extern USBD_HandleTypeDef hUsbDeviceFS;
uint8_t mouse_report[MOUSE_REPORT_SIZE] = {0};

#define INPUT_SIZE 128
char inputBuf[INPUT_SIZE];
int inputPos = 0;

void click() {
	HAL_UART_Transmit(&huart2, (uint8_t *)"Click!\r\n", 8, HAL_MAX_DELAY);

	mouse_report[0] = 0b001;
	USBD_HID_SendReport(&hUsbDeviceFS, mouse_report, MOUSE_REPORT_SIZE);
	HAL_Delay(50);

	mouse_report[0] = 0b000;
	USBD_HID_SendReport(&hUsbDeviceFS, mouse_report, MOUSE_REPORT_SIZE);
	HAL_Delay(50);
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
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n\r\nGoing to 0,0\r\n", 18, HAL_MAX_DELAY);

  for (int i = 0; i < 2048 / MOVE_DELTA; i++) {
  	  mouse_report[1] = (int8_t)-MOVE_DELTA;
  	  mouse_report[2] = (int8_t)-MOVE_DELTA;
  	  USBD_HID_SendReport(&hUsbDeviceFS, mouse_report, MOUSE_REPORT_SIZE);
  	  HAL_Delay(MOVE_DELAY);
  }

  int curx = 0;
  int cury = 0;


  HAL_UART_Transmit(&huart2, (uint8_t *)"At 0,0\r\n", 8, HAL_MAX_DELAY);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	HAL_UART_Transmit(&huart2, (uint8_t*)"Type coords: ", 13, HAL_MAX_DELAY);
	inputPos = 0;

	do {
		HAL_UART_Receive(&huart2, &inputBuf[inputPos], 1, HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart2, &inputBuf[inputPos], 1, HAL_MAX_DELAY); //echo
		inputPos++;
	}
	while (inputPos != INPUT_SIZE - 1 && inputBuf[inputPos - 1] != '\r');

	  inputBuf[inputPos] = '\0';
	  HAL_UART_Transmit(&huart2, (uint8_t*)"\n", 1, HAL_MAX_DELAY);

	  int isWheel = inputBuf[0] == 'w';

	  if (isWheel) {
		  int d = strtol(inputBuf + 1, NULL, 10);
		  if (d > 127)
			  d = 127;
		  else if (d < -128)
			  d = -128;

		  mouse_report[3] = (int8_t)d;
		  USBD_HID_SendReport(&hUsbDeviceFS, mouse_report, MOUSE_REPORT_SIZE);
		  HAL_Delay(50);
		  mouse_report[3] = 0;
	  }
	  else {
		  char *endptr = NULL;
		  int x = strtol(inputBuf, &endptr, 10);
		  int y = strtol(endptr, &endptr, 10);
		  int clickWhenMoved = strtol(endptr, NULL, 10);
		  char msgbuf[128];
		  int msglen = snprintf(msgbuf, sizeof(msgbuf), "Going to %d,%d\r\n", x, y);
		  HAL_UART_Transmit(&huart2, (uint8_t*)msgbuf, msglen, HAL_MAX_DELAY);

		  for (;;) {
			  int d[2];
			  d[0] = x - curx;
			  d[1] = y - cury;

			  if (d[0] == 0 && d[1] == 0) {
				  break;
			  }

			  for (int i=0; i < 2; i++) {
				  if (d[i] > 0 && d[i] > MOVE_DELTA) {
					  d[i] = MOVE_DELTA;
				  }
				  else if (d[i] < 0 && d[i] < -MOVE_DELTA) {
					  d[i] = -MOVE_DELTA;
				  }
			  }

			  mouse_report[1] = (int8_t)d[0];
			  mouse_report[2] = (int8_t)d[1];
			  USBD_HID_SendReport(&hUsbDeviceFS, mouse_report, MOUSE_REPORT_SIZE);
			  HAL_Delay(MOVE_DELAY);

			  curx += d[0];
			  cury += d[1];
		  }

		  mouse_report[1] = 0;
		  mouse_report[2] = 0;

		  msglen = snprintf(msgbuf, sizeof(msgbuf), "At %d,%d\r\n", x, y);
		  HAL_UART_Transmit(&huart2, (uint8_t *)msgbuf, msglen, HAL_MAX_DELAY);

          if (clickWhenMoved != 0)
		    click();
	  }
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

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

  /* USER CODE END Callback 1 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
