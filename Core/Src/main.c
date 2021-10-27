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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <lowlvl.h>
#include <SYS_access.h>
#include <ADC_access.h>
#include <PWM_access.h>
#include <I2C_access.h>
#include <ENC_access.h>
#include "ukf.h"
#include <pid.h>
#include <slam.h>
#include <algo.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define get_acc get_acceleration
#define get_w get_angular_velocity
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int encv1= 0, encv2 = 0;
microMouseState robotState;
uint8_t MAP_READY = 0;
uint8_t MAZE_COMPLETE = 0;
void initMaping() {
    //init variables
    for (int i = 0; i < MAP_SIZE_N; i++)
        for (int j = 0; j < MAP_SIZE_N; j++)
            robotState.flooding[i][j] = 0xff;
    for (int i = 0; i < MAP_SIZE_N; i++)
        for (int j = 0; j < MAP_SIZE_N; j++)
            robotState.mappedArea[i][j] = 0;
    robotState.visited[0] = TRUE;
    for (int i = 1; i <= 0xff; i++)
        robotState.visited[i] = FALSE;
    //next to explore will be initialized after first map
    robotState.step = 0;
    robotState.target = -1;
    robotState.x = 0;
    robotState.y = 0;
    robotState.rotation = 0;
    //actual visit state
    //actual sensors state
    robotState.mazeTypeRightHanded = -1;
    robotState.nodeFound = FALSE;
    robotState.pathReturn = FALSE;
    robotState.mapIsFinished = FALSE;
}

void clock_timer_start_point(){
	__HAL_TIM_SET_COUNTER(&htim5,0);
}
uint32_t clock_timer_end_point(){
	return __HAL_TIM_GET_COUNTER(&htim5);
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
extern float reference_omega;
extern float reference_velocity;
extern int request2savedata;
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
  MX_DMA_Init();
  MX_ADC3_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM12_Init();
  MX_USART3_UART_Init();
  MX_TIM14_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */
  kalman_init();
  lowlvl_init();
  initMaping();
  reference_omega = 0;
  reference_velocity = 0;
  startSystem();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // FIXME some test code
//	  if(BTN_IsPressed(1))
//	  {
//		  if(reference_omega < 10*PI)reference_omega += 1;
//		  else __NOP();
//	  }
//	  else if(BTN_IsPressed(2))
//	  {
//		  if(reference_omega > -10*PI)reference_omega -= 1;
//		  else __NOP();
//	  }
	  if(BTN_IsPressed(1))
	  {
		  if(reference_velocity < 1000)reference_velocity += 10;
		  else __NOP();
	  }
//	  else if(BTN_IsPressed(2))
//	  {
//		  if(reference_velocity > -1000)reference_velocity -= 10;
//		  else __NOP();
//	  }
	  else if(BTN_IsPressed(2))
	  {
		  if(reference_omega < 10*PI)reference_omega += 1;
		  else __NOP();
	  }
//	  battery_task();
	  //set_velocity(1,-reference_velocity);
	  //set_velocity(2,reference_velocity);
	  HAL_Delay(100);
	  switch(request2savedata)
	  {
	  	  case 1:
	  		  request2savedata = 0;
	  		  eepromSave4Bytes(&vp_addr, &vp_data);
	  		  eepromSave4Bytes(&vi_addr, &vi_data);
	  		  eepromSave4Bytes(&vd_addr, &vd_data);
	  		  printf("Parameters of PID I(v) updated\n");
	  		  break;
	  	  case 2:
	  		  request2savedata = 0;
			  eepromSave4Bytes(&wp_addr, &wp_data);
		      eepromSave4Bytes(&wi_addr, &wi_data);
			  eepromSave4Bytes(&wd_addr, &wd_data);
			  printf("Parameters of PID II(w) updated\n");
	  		  break;
	  	  default:
	  		  request2savedata = 0;
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
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

/* USER CODE BEGIN 4 */
int tim4_period_counter = 0;
extern int copy_status;
extern int mpu_is_initialized;
int mpuint_soft = TRUE;
int debug_flag = TRUE;
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
  else if (htim->Instance == TIM4) {
	  tim4_period_counter = (tim4_period_counter + 1) % 2;
	  if(tim4_period_counter)
		  uart_rx();
	  else uart_tx();
  }
  else if (htim->Instance == TIM6) {
          if (mpu_is_initialized) {
              mpuint_soft = !mpuint_soft;//
              // frequency 0.5kHz INT
              if (mpuint_soft&&debug_flag)MPUInterruptHandler();
          }
      }
  else if(htim->Instance == TIM7) {
	  if(!copy_status)kalman_filter();
  }else if(htim->Instance == TIM9) {
	  //performEncodersMeasurements();
	  pid_global();
  }else if(htim->Instance == TIM10) {
	  slam_walls_detection();
  }else if(htim->Instance == TIM13)
  {
	  static int bat_watch_cnt = 1;
	  // frequency 1kHz PWM
	  if(bat_watch_cnt == 0)
		  battery_task();
	  else
		  __NOP();
	  generateLEDPWMCallback();
	  bat_watch_cnt = (bat_watch_cnt + 1) % 1000;
  }
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
	while(1){
		ledToggle(1);
		HAL_Delay(500);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
