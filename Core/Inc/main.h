/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define XSHUT2_Pin GPIO_PIN_13
#define XSHUT2_GPIO_Port GPIOC
#define INT1_Pin GPIO_PIN_14
#define INT1_GPIO_Port GPIOC
#define INT1_EXTI_IRQn EXTI15_10_IRQn
#define XSHUT1_Pin GPIO_PIN_15
#define XSHUT1_GPIO_Port GPIOC
#define OSCIN_Pin GPIO_PIN_0
#define OSCIN_GPIO_Port GPIOH
#define OSCOUT_Pin GPIO_PIN_1
#define OSCOUT_GPIO_Port GPIOH
#define UNUSED8_Pin GPIO_PIN_0
#define UNUSED8_GPIO_Port GPIOC
#define UNUSED9_Pin GPIO_PIN_1
#define UNUSED9_GPIO_Port GPIOC
#define UNUSED10_Pin GPIO_PIN_2
#define UNUSED10_GPIO_Port GPIOC
#define UNUSED11_Pin GPIO_PIN_3
#define UNUSED11_GPIO_Port GPIOC
#define EN11_Pin GPIO_PIN_0
#define EN11_GPIO_Port GPIOA
#define EN12_Pin GPIO_PIN_1
#define EN12_GPIO_Port GPIOA
#define UNUSED1_Pin GPIO_PIN_2
#define UNUSED1_GPIO_Port GPIOA
#define BATLVL_Pin GPIO_PIN_3
#define BATLVL_GPIO_Port GPIOA
#define BTN1_Pin GPIO_PIN_4
#define BTN1_GPIO_Port GPIOA
#define UNUSED2_Pin GPIO_PIN_5
#define UNUSED2_GPIO_Port GPIOA
#define BTN2_Pin GPIO_PIN_6
#define BTN2_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_7
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOC
#define RX_Pin GPIO_PIN_5
#define RX_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_0
#define LED3_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_1
#define LED4_GPIO_Port GPIOB
#define LED5_Pin GPIO_PIN_2
#define LED5_GPIO_Port GPIOB
#define TX_Pin GPIO_PIN_10
#define TX_GPIO_Port GPIOB
#define UNUSED6_Pin GPIO_PIN_12
#define UNUSED6_GPIO_Port GPIOB
#define UNUSED7_Pin GPIO_PIN_13
#define UNUSED7_GPIO_Port GPIOB
#define PWMB_Pin GPIO_PIN_14
#define PWMB_GPIO_Port GPIOB
#define PWMA_Pin GPIO_PIN_15
#define PWMA_GPIO_Port GPIOB
#define EN22_Pin GPIO_PIN_6
#define EN22_GPIO_Port GPIOC
#define EN21_Pin GPIO_PIN_7
#define EN21_GPIO_Port GPIOC
#define UNUSED12_Pin GPIO_PIN_8
#define UNUSED12_GPIO_Port GPIOC
#define BBAC_Pin GPIO_PIN_9
#define BBAC_GPIO_Port GPIOC
#define BFOR_Pin GPIO_PIN_8
#define BFOR_GPIO_Port GPIOA
#define UNUSED3_Pin GPIO_PIN_9
#define UNUSED3_GPIO_Port GPIOA
#define ABAC_Pin GPIO_PIN_10
#define ABAC_GPIO_Port GPIOA
#define AFOR_Pin GPIO_PIN_11
#define AFOR_GPIO_Port GPIOA
#define UNUSED4_Pin GPIO_PIN_12
#define UNUSED4_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWDCK_Pin GPIO_PIN_14
#define SWDCK_GPIO_Port GPIOA
#define UNUSED5_Pin GPIO_PIN_15
#define UNUSED5_GPIO_Port GPIOA
#define UNUSED13_Pin GPIO_PIN_10
#define UNUSED13_GPIO_Port GPIOC
#define INT5_Pin GPIO_PIN_11
#define INT5_GPIO_Port GPIOC
#define INT5_EXTI_IRQn EXTI15_10_IRQn
#define XSHUT5_Pin GPIO_PIN_12
#define XSHUT5_GPIO_Port GPIOC
#define MPUINT_Pin GPIO_PIN_2
#define MPUINT_GPIO_Port GPIOD
#define MPUINT_EXTI_IRQn EXTI2_IRQn
#define INT4_Pin GPIO_PIN_3
#define INT4_GPIO_Port GPIOB
#define INT4_EXTI_IRQn EXTI3_IRQn
#define XSHUT4_Pin GPIO_PIN_4
#define XSHUT4_GPIO_Port GPIOB
#define INT3_Pin GPIO_PIN_5
#define INT3_GPIO_Port GPIOB
#define INT3_EXTI_IRQn EXTI9_5_IRQn
#define SCL_Pin GPIO_PIN_6
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_7
#define SDA_GPIO_Port GPIOB
#define XSHUT3_Pin GPIO_PIN_8
#define XSHUT3_GPIO_Port GPIOB
#define INT2_Pin GPIO_PIN_9
#define INT2_GPIO_Port GPIOB
#define INT2_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */
typedef enum{FALSE=0,TRUE=1}boolean;
#include "arm_math.h"
typedef struct{
    arm_matrix_instance_f32 *mat;
    char friendly_name;
}matrix;
void clock_timer_start_point();
uint32_t clock_timer_end_point();
void startSystem();
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
