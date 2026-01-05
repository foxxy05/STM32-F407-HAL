/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CHASSIS_PWM 50
#define CLIMBING_PWM 50
#define STOP_PWM 0
#define maxPWM 60
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// MOTOR VARIABLES FOR CCR REGISTERS
volatile uint32_t *const LF_LPWM_REG = &(TIM3->CCR3);
volatile uint32_t *const LF_RPWM_REG = &(TIM3->CCR4);

volatile uint32_t *const RF_LPWM_REG = &(TIM3->CCR2);
volatile uint32_t *const RF_RPWM_REG = &(TIM3->CCR1);

volatile uint32_t *const LR_LPWM_REG = &(TIM9->CCR1);
volatile uint32_t *const LR_RPWM_REG = &(TIM9->CCR2);

volatile uint32_t *const RR_LPWM_REG = &(TIM4->CCR3);
volatile uint32_t *const RR_RPWM_REG = &(TIM4->CCR4);

volatile uint32_t *const LiftFront_LPWM_REG = &(TIM12->CCR1);
volatile uint32_t *const LiftFront_RPWM_REG = &(TIM12->CCR2);

volatile uint32_t *const LiftRear_LPWM_REG = &(TIM13->CCR1);
volatile uint32_t *const LiftRear_RPWM_REG = &(TIM14->CCR1);

// MOTOR VARIABLES FOR TIMER CHANNELS
#define LF_LPWM 3
#define LF_RPWM 4
#define RF_LPWM 2
#define RF_RPWM 1
#define LR_LPWM 1
#define LR_RPWM 2
#define RR_LPWM 3
#define RR_RPWM 4
#define LiftFront_LPWM 1
#define LiftFront_RPWM 2
#define LiftRear_LPWM  1
#define LiftRear_RPWM  1


// DATA VARIABLES
#define buffer 10
#define arraySize 5
#define slaveAddr 0x08
uint8_t receivedData[arraySize] = { 0 }; //LX LY L2 R2 Button (LB, LF, LR, DB, 0, 0, 0, 0)
int8_t lx = 0, ly = 0;
uint8_t l2 = 0, r2 = 0;

// NAVIGATION VARIABLES
#define constVector 1
int16_t wLF = 0, wRF = 0, wLR = 0, wRR = 0;
int16_t Vx = 0, Vy = 0;
int16_t VxG = 0, VyG = 0;
int16_t omega = 0;
// CLIMBING VARIABLES
bool liftBoth  = false;
bool liftFront = false;
bool liftRear  = false;
bool dropBoth  = false;

// PID
//float currentTime = 0;
//float previousTime = 0;
//float error = 0;
//float previousError = 0;
//float derivative = 0;
//float kp = 4;
//float kd = 0;
//float PID = 0;
//int targetAngle = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM14_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// PRINT-PS FUNCTION
void printPS(){
    char bufferMsg[50];
    // Format: "LX: 10, LY: -5, L2: 0, R2: 0, Btn: 128\r\n"
    int len = sprintf(bufferMsg, "LX:%d  LY:%d L2:%d  R2:%d  BTN:%d\r\n",
                      receivedData[0], receivedData[1], receivedData[2],
                      receivedData[3], receivedData[4]);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);
}

// CONSTRAIN FUNCTION
int constrain(int val, int min, int max){
	if(val >= max)			val = max;
	else if (val <= min) 	val = min;
	else 					val = val;

	return val;
}

// MAP FUNCTION
long map(long val, long oldMin, long oldMax, long newMin, long newMax){
	return (val - oldMin) * (newMax  - newMin) / (oldMax - oldMin) + newMin;
}

void reMapComm(){
	receivedData[0] = map(receivedData[0], 0, 255, -127, 127);
	receivedData[1] = map(receivedData[1], 0, 255, -127, 127);
	receivedData[2] = receivedData[2];
	receivedData[3] = receivedData[3];
}

void LFRotate(int PWM){
	if(PWM < 0){
		*LF_LPWM_REG = abs(PWM);
		*LF_RPWM_REG = 0;
	}
	else if(PWM > 0){
		*LF_LPWM_REG = 0;
		*LF_RPWM_REG = abs(PWM);
	}
	else{
		*LF_LPWM_REG = 0;
		*LF_RPWM_REG = 0;
	}
}

void RFRotate(int PWM){
	if(PWM < 0){
		*RF_LPWM_REG = abs(PWM);
		*RF_RPWM_REG = 0;
	}
	else if(PWM > 0){
		*RF_LPWM_REG = 0;
		*RF_RPWM_REG = abs(PWM);
	}
	else{
		*RF_LPWM_REG = 0;
		*RF_RPWM_REG = 0;
	}
}

void LRRotate(int PWM){
	if(PWM < 0){
		*LR_LPWM_REG = abs(PWM);
		*LR_RPWM_REG = 0;
	}
	else if(PWM > 0){
		*LR_LPWM_REG = 0;
		*LR_RPWM_REG = abs(PWM);
	}
	else{
		*LR_LPWM_REG = 0;
		*LR_RPWM_REG = 0;
	}
}

void RRRotate(int PWM){
	if(PWM < 0){
		*RR_LPWM_REG = abs(PWM);
		*RR_RPWM_REG = 0;
	}
	else if(PWM > 0){
		*RR_LPWM_REG = 0;
		*RR_RPWM_REG = abs(PWM);
	}
	else{
		*RR_LPWM_REG = 0;
		*RR_RPWM_REG = 0;
	}
}

void liftFrontMotor(int PWM){
	if(PWM < 0){
		*LiftFront_LPWM_REG = abs(PWM);
		*LiftFront_RPWM_REG = 0;
	}
	else if(PWM > 0){
		*LiftFront_LPWM_REG = 0;
		*LiftFront_RPWM_REG = abs(PWM);
	}
	else{
		*LiftFront_LPWM_REG = 0;
		*LiftFront_RPWM_REG = 0;
	}
}

void liftRearMotor(int PWM){
	if(PWM < 0){
		*LiftRear_LPWM_REG = abs(PWM);
		*LiftRear_RPWM_REG = 0;
	}
	else if(PWM > 0){
		*LiftRear_LPWM_REG = 0;
		*LiftRear_RPWM_REG = abs(PWM);
	}
	else{
		*LiftRear_LPWM_REG = 0;
		*LiftRear_RPWM_REG = 0;
	}
}

void liftBothMotor(int PWM){
	liftFrontMotor(PWM);
	liftRearMotor(PWM);
}

// PID CONTROL FUNCTION
//float PIDControl(int error) {
//  currentTime = millis();
//  int deltaT = (currentTime - previousTime);
//  if (deltaT <= 0) {
//    deltaT = 1;
//  }
//  derivative = (error - previousError) / (deltaT);
//  PID = kp * error + kd * derivative;
//  previousError = error;
//  previousTime = currentTime;
//  PID = constrain(PID, -maxPWM, maxPWM);
//  if (abs(PID) <= 1) {
//    PID = 0;
//  }
//  return PID;
//}



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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // TIMER-3 Enable
  HAL_TIM_PWM_Start(&htim3, LF_LPWM);
  HAL_TIM_PWM_Start(&htim3, LF_RPWM);
  HAL_TIM_PWM_Start(&htim3, RF_LPWM);
  HAL_TIM_PWM_Start(&htim3, RF_RPWM);

  // TIMER-4 Enable
  HAL_TIM_PWM_Start(&htim4, RR_LPWM);
  HAL_TIM_PWM_Start(&htim4, RR_RPWM);

  // TIMER-9 Enable
  HAL_TIM_PWM_Start(&htim9, LR_LPWM);
  HAL_TIM_PWM_Start(&htim9, LR_RPWM);

  // TIMER-12 Enable
  HAL_TIM_PWM_Start(&htim12, LiftFront_LPWM);
  HAL_TIM_PWM_Start(&htim12, LiftFront_RPWM);

  // TIMER-13 Enable
  HAL_TIM_PWM_Start(&htim13, LiftRear_LPWM);
  // TIMER-14 Enable
  HAL_TIM_PWM_Start(&htim14, LiftRear_RPWM);


//  TIM3 -> CCR1 = CHASSIS_PWM;	//RF_RPWM
//  TIM3 -> CCR2 = CHASSIS_PWM;	//RF_LWPM
//  TIM3 -> CCR3 = CHASSIS_PWM;	//LF_LPWM
//  TIM3 -> CCR4 = CHASSIS_PWM;	//LF_RPWM
//
//  TIM4 -> CCR3 = CHASSIS_PWM;	//RR_LPWM
//  TIM4 -> CCR4 = CHASSIS_PWM;	//RR_RPWM
//
//  TIM9 -> CCR1 = CHASSIS_PWM;	//LR_LPWM
//  TIM9 -> CCR2 = CHASSIS_PWM;	//LR_RPWM
//
//  TIM12 -> CCR1 = CLIMBING_PWM;	//LiftFront_LPWM
//  TIM12 -> CCR2 = CLIMBING_PWM;	//LiftFront_RPWM
//
//  TIM13 -> CCR1 = CLIMBING_PWM;	//LiftRear_LPWM
//  TIM14 -> CCR1 = CLIMBING_PWM;	//LiftRear_RPWM

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  currentAngle = 0;
	  wLF = 0;
	  wRF = 0;
	  wLR = 0;
	  wRR = 0;
	  Vx = 0, Vy = 0;
	  omega = 0;
	  liftBoth = liftFront = liftRear = dropBoth = false;

	  // COMMUNICATION
	  HAL_I2C_Master_Receive(&hi2c2, slaveAddr, receivedData, sizeof(receivedData), HAL_MAX_DELAY);
	  reMapComm();

	  Vy = receivedData[0];  //Y-Component of the Joy stick is the X component of the Chassis
	  Vx = receivedData[1];
	  omega = receivedData[2] - receivedData[3];

	  liftBoth  = (receivedData[4] & 0b10000000) ? 1 : 0;
	  liftFront = (receivedData[4] & 0b01000000) ? 1 : 0;
	  liftRear  = (receivedData[4] & 0b00100000) ? 1 : 0;
	  dropBoth  = (receivedData[4] & 0b00010000) ? 1 : 0;

	  VxG = Vx;
	  VyG = Vy;

	  // GLOBAL-CONTROL
//	  VxG = Vx * cos(-theta) - Vy * sin(-theta);  // Local X
//	  VyG = Vx * sin(-theta) + Vy * cos(-theta);  // Local Y

	  wLF = constrain(constVector * (VxG + VyG - omega), -maxPWM, maxPWM);
	  wRF = constrain(constVector * (VxG - VyG + omega), -maxPWM, maxPWM);
	  wLR = constrain(constVector * (VxG - VyG - omega), -maxPWM, maxPWM);
	  wRR = constrain(constVector * (VxG + VyG + omega), -maxPWM, maxPWM);

	  LFRotate(wLF);
	  RFRotate(wRF);
	  LRRotate(wLR);
	  RRRotate(wRR);

	  if(liftFront){
		  liftFrontMotor(CLIMBING_PWM);
		  liftRearMotor(STOP_PWM);
	  }
	  else if(liftRear){
		  liftFrontMotor(STOP_PWM);
		  liftRearMotor(CLIMBING_PWM);
	  }
	  else if(liftBoth){
		  liftFrontMotor(CLIMBING_PWM);
	  	  liftRearMotor(CLIMBING_PWM);
	  }
	  else if(dropBoth){
		  liftFrontMotor(-CLIMBING_PWM);
		  liftRearMotor(-CLIMBING_PWM);
	  }
	  else{
		  liftFrontMotor(STOP_PWM);
		  liftRearMotor(STOP_PWM);
	  }


	 printPS();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 20;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 20;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 41;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 99;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 20;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 99;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 20;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 99;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */
  HAL_TIM_MspPostInit(&htim13);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 20;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 99;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
