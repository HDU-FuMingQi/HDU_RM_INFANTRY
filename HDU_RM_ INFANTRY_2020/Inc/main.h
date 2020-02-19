/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "kalman.h"
#include "stm32f4xx_hal_can.h"
#include "stdbool.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "led.h"
#include "bsp_pid.h"
#include "bsp_can.h"
#include "myType.h"
#include "bsp_dbus.h"
#include "bsp_imu.h"
#include "mahony_ahrs.h"
#include "tim.h"
#include "ICM20602.h"
#include "CRC.h"
#include "jy901.h"
#include "mysystem.h"
#include "OLED_Task.h"
#include "bsp_oled.h"
#include "mytype.h"
#include "bsp_motor.h"
#include "myparam.h"
#include "cmsis_os.h"
#include "math.h"
#include "stdlib.h"
#include "string.h"
#include "shoot_task.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "bsp_vision.h"
#include "shoot_task.h"
#include "Hero_shoot_Task.h"
////////////////µ¯²Ö//////////////
#include "magazine.h"
///////////////Peripheral///////////
#include "control.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern __IO uint16_t uhADCxConvertedValue[30];
extern uint16_t ADCxConvertedValue;
#define PI 3.141592654f
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
#define IST_RESET_Pin GPIO_PIN_2
#define IST_RESET_GPIO_Port GPIOE
#define SPI5_NSS_Pin GPIO_PIN_6
#define SPI5_NSS_GPIO_Port GPIOF
#define key_signal_Pin GPIO_PIN_10
#define key_signal_GPIO_Port GPIOF
#define POWER1_Pin GPIO_PIN_2
#define POWER1_GPIO_Port GPIOH
#define POWER2_Pin GPIO_PIN_3
#define POWER2_GPIO_Port GPIOH
#define POWER3_Pin GPIO_PIN_4
#define POWER3_GPIO_Port GPIOH
#define POWER4_Pin GPIO_PIN_5
#define POWER4_GPIO_Port GPIOH
#define LED3_Pin GPIO_PIN_3
#define LED3_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_14
#define LED1_GPIO_Port GPIOF
#define LED2_Pin GPIO_PIN_11
#define LED2_GPIO_Port GPIOE
#define OLED_RST_Pin GPIO_PIN_10
#define OLED_RST_GPIO_Port GPIOB
#define Buzzer_Pin GPIO_PIN_6
#define Buzzer_GPIO_Port GPIOH
#define POWER5_Pin GPIO_PIN_2
#define POWER5_GPIO_Port GPIOI
#define CRX_Pin GPIO_PIN_0
#define CRX_GPIO_Port GPIOD
#define CTX_Pin GPIO_PIN_1
#define CTX_GPIO_Port GPIOD
#define OLED_DC_Pin GPIO_PIN_9
#define OLED_DC_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define VAL_MIN(a, b) ((a) < (b) ? (a) : (b))
#define VAL_MAX(a, b) ((a) > (b) ? (a) : (b))
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
