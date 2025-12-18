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
#include <stdio.h>
#include <string.h>
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t const BNO055Addr = 0X28;				 	//Address
uint8_t const BNO055I2CAddr = BNO055Addr << 1;	//Address
uint8_t const BNO055MemAddr = 0X00;				//Address
uint8_t BNO055IDValue = 0xA0;

uint8_t BNO055OprModeRegAddr = 0X3D;		//Address
uint8_t BNO055ConfigMode = 0X00;
uint8_t BNO055NDOFMode = 0X0C;

#define dataSize 6
uint8_t const BNO055EulerAnchorAddr = 0X1A;		//Address
uint8_t rawData[dataSize];
int16_t rawH, rawR, rawP;
float heading, roll, pitch;
char UART_Buffer[100];

uint8_t const resetCommandTrigRegAddr = 0X3F;
uint8_t resetCommand = 0X20;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  uint8_t chipID = 0;
  char msg[50];

  HAL_StatusTypeDef Status = HAL_I2C_Mem_Read(&hi2c1, BNO055I2CAddr, BNO055MemAddr, I2C_MEMADD_SIZE_8BIT, &chipID, sizeof(chipID), HAL_MAX_DELAY);

  if (Status == HAL_OK && chipID == BNO055IDValue) {
	  sprintf(msg, "BNO055 Found! ID: 0x%02X\r\n", chipID);
  }
  else{
	  sprintf(msg, "BNO055 Not Found. Error or ID: 0x%02X\r\n", chipID);
  }

  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
//  Entering the Config Mode
  HAL_I2C_Mem_Write(&hi2c1, BNO055I2CAddr, BNO055OprModeRegAddr, I2C_MEMADD_SIZE_8BIT, &BNO055ConfigMode, sizeof(BNO055ConfigMode), HAL_MAX_DELAY);
  HAL_Delay(20);		//Specific delay mentioned for Config Mode according to the BNO055 Datasheet
  HAL_I2C_Mem_Write(&hi2c1, BNO055I2CAddr, BNO055OprModeRegAddr, I2C_MEMADD_SIZE_8BIT, &BNO055NDOFMode, sizeof(BNO055NDOFMode), HAL_MAX_DELAY);
  HAL_Delay(10);		//Specific delay mentioned for NDOF Mode according to the BNO055 Datasheet

  HAL_Delay(2000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_StatusTypeDef readStatus = HAL_I2C_Mem_Read(&hi2c1, BNO055I2CAddr, BNO055EulerAnchorAddr, I2C_MEMADD_SIZE_8BIT, rawData, dataSize, HAL_MAX_DELAY);
	  if(readStatus == HAL_OK){
		  rawH = (int16_t)((rawData[1] << 8) | rawData[0]);
		  rawR = (int16_t)((rawData[3] << 8) | rawData[2]);
		  rawP = (int16_t)((rawData[5] << 8) | rawData[4]);

		  heading = (float) rawH / 16.0f;
		  roll = (float) rawR / 16.0f;
		  pitch = (float) rawP / 16.0f;

		  sprintf(UART_Buffer, "H: %f | R: %f | P: %f\r\n", heading, roll, pitch);
		  HAL_UART_Transmit(&huart2, (uint8_t*)UART_Buffer, strlen(UART_Buffer), 10);
	  }
	  else {
	      // REINITIALIZATION LOGIC
	      sprintf(msg, "BNO055 Not Found. Attempting Reset...\r\n");
	      HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
	      HAL_Delay(200);

	      // Write to SYS_TRIGGER (0x3F)
	      HAL_I2C_Mem_Write(&hi2c1, BNO055I2CAddr, resetCommandTrigRegAddr, I2C_MEMADD_SIZE_8BIT, &resetCommand, sizeof(resetCommand), 100);

	      // Critical Delay: The datasheet says wait 650ms after reset
	      HAL_Delay(700);

	      // Second Attempt
	      HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, BNO055I2CAddr, BNO055MemAddr, I2C_MEMADD_SIZE_8BIT, &chipID, 1, 100);

	      if (status == HAL_OK && chipID == BNO055IDValue) {
	          sprintf(msg, "BNO055 Recovered! ID: 0x%02X\r\n", chipID);
	      } else {
	          sprintf(msg, "BNO055 Hard Failure. Check Wiring/Power.\r\n");
	          // Optional: Stay here or enter Error_Handler()
	      }
	      HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
	  }

	  HAL_Delay(100);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	  // Configure I2C1 Pins: PB6 (SCL) and PB7 (SDA)
	  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;      // Alternate Function Open Drain
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;  // AF4 is I2C for F411
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
