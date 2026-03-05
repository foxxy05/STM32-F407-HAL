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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include<stdbool.h>
#include<stdint.h>
#include<string.h>
#include<math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define BNO055_ADDR (0x28 << 1)
#define PWR_MODE 0x3E
#define OPR_MODE 0x3D
#define EUL_DATA_START 0x1A

uint8_t bno_data[6];
float current_heading, current_roll, current_pitch;

#define data_size 5
#define slave_addr 8
uint8_t data[data_size];
char buffer[100];

int L_joystick_x = 0, L_joystick_y = 0, R_joystick_x = 0, R_joystick_y = 0;

#define radius       1
#define length       1
#define MAX_PWM      60
#define DEADZONE     35
#define dt           0.001

uint8_t L2;
uint8_t R2;
bool L1;
bool R1;

bool right;
bool down;
bool up;
bool left;
bool square;
bool cross;
bool circle;
bool triangle;

int pwm1, pwm2, pwm3;

float bot_vel, theta;
int16_t omega;
  bool flag = false;
bool moving = false;
bool is_omega = false;
  bool pid_tim = false;

float P,I,D;
float p,i,d;

float kp = 0;
float ki = 0;
float kd = 0;
float KP = 2.5;
float KI = 0;
float KD = 0.5;

float k = 0.0015;
float error, error1, error2, d_error;
float ERR, ERR1, ERR2, D_error;
float correction;
float CORRECTION;
float target_heading = 0;

 bool pid = true;

 unsigned long  t_start, t_end;
 float dt2;

volatile bool imu = false;
volatile bool value = false;

uint32_t i2c1, i2c2;



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM5_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

CAN_FilterTypeDef sFilterConfig;
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

uint32_t blink_delay;

uint8_t msg[150];


float radians(float angle){
  	return angle*2*3.14159265358979323846/360.0;
  }

  float degrees(float angle){
  	return angle*360.0/(2*3.14159265358979323846);
  }

  long map(long x, long in_min, long in_max, long out_min, long out_max)
  {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  float constrain(float num, float min, float max){

  	if(num > max) {
  		return max;
  	}

  	else if(num < min){
  		return min;
  	}

  	else {
  		return num;
  	}
  }


  void BNO055_init(){

   uint8_t pwr = 0x00;
   uint8_t opr = 0x0C;
   uint8_t config = 0x00;

    HAL_I2C_Mem_Write(&hi2c2, BNO055_ADDR, OPR_MODE, 1, &config, 1, 100); // setting to config mode
  	HAL_I2C_Mem_Write(&hi2c2, BNO055_ADDR, PWR_MODE, 1, &pwr, 1, 100);  // setting bno055 to normal mode
  	HAL_I2C_Mem_Write(&hi2c2, BNO055_ADDR, OPR_MODE, 1, &opr, 1, 100); // setting to Ndof mode

  	//HAL_Delay(700);

  }

  float normalise_angle(float angle)
  {
      if(angle > 180.0)  {
    	  angle -= 360.0;
      }
      if(angle < -180.0) {
    	  angle += 360.0;
      }
      return angle;
  }


  void get_angle(){

  	  //HAL_I2C_Mem_Read(&hi2c2, BNO055_ADDR,EUL_DATA_START , 1, bno_data, 6, 100);
//      current_heading = (bno_data[0] | (bno_data[1] << 8)/16);
//      current_roll = bno_data[2] | (bno_data[3] << 8)/16;
//      current_pitch = bno_data[4] | (bno_data[5] << 8)/16;


//  	int16_t heading_raw = (int16_t)(bno_data[0] | (bno_data[1] << 8));
//  	int16_t roll_raw    = (int16_t)(bno_data[2] | (bno_data[3] << 8));
//  	int16_t pitch_raw   = (int16_t)(bno_data[4] | (bno_data[5] << 8));
//
//  	    current_heading = normalise_angle(heading_raw / 16.0f);
//  	    current_roll    = roll_raw / 16.0f;
//  	    current_pitch   = pitch_raw / 16.0f;

//      char buffer1[100];
//
//    		    		      sprintf(buffer1, "heading = %d \t roll = %d \t pitch = %d \r\n",  current_heading, current_roll, current_pitch);
//    		    		 	  HAL_UART_Transmit(&huart2, (uint8_t*)buffer1, strlen(buffer1), 100);



  	 if(HAL_I2C_Mem_Read(&hi2c2, BNO055_ADDR, EUL_DATA_START, 1, bno_data, 6, 100) == HAL_OK){

  	    int16_t heading_raw = (int16_t)((bno_data[1] << 8) | bno_data[0]);
  	    current_heading = heading_raw / 16.0;

  	    current_heading = -normalise_angle(current_heading);
  	 }

  	 else{

//  		 uint8_t resetCommand = 0x20;
//  		 uint8_t resetCommandTrigRegAddr = 0x3F;
//
//  	             	i2c2 = HAL_I2C_GetError(&hi2c2);
//  		            HAL_I2C_DeInit(&hi2c2);
//  		  		    HAL_Delay(50);
//  		  		    HAL_I2C_Init(&hi2c2);
//  		  		    HAL_Delay(100);
//
//  		     // Write to SYS_TRIGGER (0x3F)
//  		     HAL_I2C_Mem_Write(&hi2c2, BNO055_ADDR, resetCommandTrigRegAddr, I2C_MEMADD_SIZE_8BIT, &resetCommand, sizeof(resetCommand), 100);
//
//  		     // Critical Delay: The datasheet says wait 650ms after reset
//  		     HAL_Delay(700);
//  		     BNO055_init();
//

  	 }

  }

  void decode_buttons(){

      L_joystick_x = map(RxData[0], 0, 255, 127, -127);
     		  L_joystick_y = map(RxData[1], 0, 255, 127, -127);

     		    	right = ((RxData[2] & (1 << 0)) ? 1 : 0);
     		       down = ((RxData[2] & (1 << 1)) ? 1 : 0);
     		    	     up = ((RxData[2] & (1 << 2)) ? 1 : 0);
     		    	     left = ((RxData[2] & (1 << 3)) ? 1 : 0);
     		    	     square = ((RxData[2] & (1 << 4)) ? 1 : 0);
     		    	     cross = ((RxData[2] & (1 << 5)) ? 1 : 0);
     		    	     circle = ((RxData[2] & (1 << 6)) ? 1 : 0);
     		    	     triangle = ((RxData[2] & (1 << 7)) ? 1 : 0);

     		    	     L2 = RxData[3];
     		    	     R2 = RxData[4];


     		    	     omega = map(L2-R2, -255, 255, 40, -40);

     		    	   if(abs(L_joystick_x) < DEADZONE)
     		    	 	  {
     		    	 		  L_joystick_x = 0;
     		    	 	  }
     		    	 	  if(abs(L_joystick_y) < DEADZONE)
     		    	 		  {
     		    	 			  L_joystick_y = 0;
     		    	 		  }
     		    	 	  if(abs(omega) < DEADZONE)
     		    	 		  {
     		    	 			  omega = 0;
     		    	 		  }

  }
  void find_wheel_vel(){

	  bot_vel = sqrt( pow(L_joystick_x, 2) + pow( L_joystick_y, 2) );
	  theta = atan2(L_joystick_y, L_joystick_x) - radians(target_heading);

//	  pwm1 = constrain(((bot_vel*cos(theta) - (length*omega)) / radius) + correction, -MAX_PWM, MAX_PWM);
//	  pwm2 = constrain(((sqrt(3)*bot_vel*sin(theta) - bot_vel*cos(theta) - 2*length*omega) / (2*radius)) + correction, -MAX_PWM, MAX_PWM);
//	  pwm3 = constrain((-1*((bot_vel*cos(theta) + sqrt(3)*bot_vel*sin(theta) + 2*length*omega) / (2*radius))) + correction, -MAX_PWM, MAX_PWM);
//

	  pwm1 = constrain(((bot_vel*cos(theta) - (length*omega)) / radius) + CORRECTION, -MAX_PWM, MAX_PWM);
	  pwm2 = constrain(((sqrt(3)*bot_vel*sin(theta) - bot_vel*cos(theta) - 2*length*omega) / (2*radius)) + CORRECTION, -MAX_PWM, MAX_PWM);
	  pwm3 = constrain((-1*((bot_vel*cos(theta) + sqrt(3)*bot_vel*sin(theta) + 2*length*omega) / (2*radius))) + CORRECTION, -MAX_PWM, MAX_PWM);

	  //target_heading += omega*k;

//	  char buffer1[100];
//
//	  		    		      sprintf(buffer1, "pwm1 = %d \t pwm2 = %d \t pwm3 = %d \r\n",  pwm1, pwm2, pwm3);
//	  		    		 	  HAL_UART_Transmit(&huart2, (uint8_t*)buffer1, strlen(buffer1), 100);

  }

  void motor1(int speed){

	  if(speed > 0){
		  TIM3->CCR1 = speed;          //PA6
		  TIM3->CCR2 = 0;               //PA7

	  }

	  else{
		  TIM3->CCR1 = 0;
		  TIM3->CCR2 = -1*speed;

	  }

  }

  void motor2(int speed){

 	  if(speed > 0){
 		  TIM3->CCR3 = speed;          //PB0
 		  TIM3->CCR4 = 0;                //PB1

 	  }

 	  else{
 		  TIM3->CCR3 = 0;
 		  TIM3->CCR4 = -1*speed;

 	  }

   }

  void motor3(int speed){

 	  if(speed > 0){
 		  TIM9->CCR1 = speed;     //PE5
 		  TIM9->CCR2 = 0;         //PE6
 	  }

 	  else{
 		  TIM9->CCR1 = 0;
 		  TIM9->CCR2 = -1*speed;

 	  }

   }

  void PID(){

//   error = normalise_angle(target_heading - current_heading);
//   d_error = error - error2;
//   error2 = error;
//
//  P = error*kp;
//  I = I + (error*dt*ki);
//  D = (d_error/dt)*kd;
//
//  correction = P+I+D;

  }


  void rotate_motor(){

	      motor1(pwm1);
	      motor2(pwm2);
	      motor3(pwm3);


//	      sprintf(msg, "%d    %d   %d   %f \r\n", pwm1, pwm2, pwm3, current_heading);
//	           CDC_Transmit_FS(msg, sizeof(msg));


  }

  void rotate_motor2(){

 	      motor1(constrain(correction, -70, 70));
 	      motor2(constrain(correction, -70, 70));
 	      motor3(constrain(correction, -70, 70));


   }


  void PID2(){

	  ERR = normalise_angle(target_heading - current_heading);
	   D_error = ERR - ERR2;
	   ERR2 = ERR;

	  p = ERR*KP;
	  i = i + (ERR*(float)dt2*KI);
	  d = (D_error/(float)dt2)*KD;

	  CORRECTION = p+i+d;


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
  MX_CAN1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_TIM9_Init();
  MX_TIM5_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;                      //0x123 << 5 to accept only id 123
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;                  // 0x7FF << 5 to accept only id 123
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;

  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);

  // THEN start CAN
  HAL_CAN_Start(&hcan1);

  // THEN activate interrupt
   HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

   HAL_TIM_Base_Start_IT(&htim5);
   HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
   BNO055_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


//      sprintf(msg, "%d    %d   %d    %d    %d\r\n", RxData[0], RxData[1], RxData[2], RxData[3], RxData[4]);
//      CDC_Transmit_FS(msg, sizeof(msg));


	  t_start = HAL_GetTick();
	    dt2 = (t_start - t_end) / 1000.0;
	    t_end = t_start;

	      if(value){
            decode_buttons();
                 value = false;
	      }

	      if(imu){
		  get_angle();
		  imu = false;
	      }
//
//		  if(abs(L_joystick_x) < DEADZONE)
//		  {
//			  L_joystick_x = 0;
//		  }
//		  if(abs(L_joystick_y) < DEADZONE)
//			  {
//				  L_joystick_y = 0;
//			  }
//		  if(abs(omega) < DEADZONE)
//			  {
//				  omega = 0;
//			  }

		  if(omega){
				  		  pid = false;
				  		  flag = true;

				  	  }

				  	 if(!omega && flag){
				  		 flag = false;
				  		 pid = true;
				  		 target_heading = current_heading;
				  	 }

				  	if(pid){
				  			 PID2();
				  	}

		 find_wheel_vel();
		 rotate_motor();

		 	      sprintf(msg, "%d    %f \r\n" ,pid , current_heading);
		 	           CDC_Transmit_FS(msg, sizeof(msg));


     //HAL_Delay(5);



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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 325;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 255;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 8400-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 100-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 650;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 255;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */
  if(htim->Instance == TIM5){

	  imu = true;

  }
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
        if(RxHeader.StdId == 0x123)
        {
            value = true;

        }
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
