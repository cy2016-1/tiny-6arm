/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED0_Pin GPIO_PIN_9
#define LED0_GPIO_Port GPIOF
#define LED1_Pin GPIO_PIN_10
#define LED1_GPIO_Port GPIOF
#define STEP6_DIR_Pin GPIO_PIN_0
#define STEP6_DIR_GPIO_Port GPIOG
#define STEP6_PUL_Pin GPIO_PIN_1
#define STEP6_PUL_GPIO_Port GPIOG
#define STEP1_DIR_Pin GPIO_PIN_7
#define STEP1_DIR_GPIO_Port GPIOE
#define STEP1_PUL_Pin GPIO_PIN_8
#define STEP1_PUL_GPIO_Port GPIOE
#define STEP2_DIR_Pin GPIO_PIN_9
#define STEP2_DIR_GPIO_Port GPIOE
#define STEP2_PUL_Pin GPIO_PIN_10
#define STEP2_PUL_GPIO_Port GPIOE
#define STEP3_DIR_Pin GPIO_PIN_11
#define STEP3_DIR_GPIO_Port GPIOE
#define STEP3_PUL_Pin GPIO_PIN_12
#define STEP3_PUL_GPIO_Port GPIOE
#define STEP4_DIR_Pin GPIO_PIN_13
#define STEP4_DIR_GPIO_Port GPIOE
#define STEP4_PUL_Pin GPIO_PIN_14
#define STEP4_PUL_GPIO_Port GPIOE
#define STEP5_DIR_Pin GPIO_PIN_15
#define STEP5_DIR_GPIO_Port GPIOE
#define STEP5_PUL_Pin GPIO_PIN_10
#define STEP5_PUL_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

uint16_t adc_data[10];
#define rx_bufSize 256
uint8_t rx_data[rx_bufSize];
uint16_t rx_Count;

extern uint32_t count;
extern int16_t instruct[500][11];//模式，条件|输出，�?�度|加�?�度，位置，时间�?????
extern int instruct_step;
extern int total_step;
extern float step;
extern float step_speed;
extern float step_acc;
extern float step_time;
extern char is_start;
extern char is_stop;
extern char is_single_run;
//电机1参数
extern int tar_pulse1;
extern int cur_pulse1;
extern int pre_pulse1;
extern int tar_speed1;
extern int cur_speed1;
extern int tar_acc1;
extern int cur_acc1;

//电机2参数
extern int tar_pulse2;
extern int cur_pulse2;
extern int pre_pulse2;
extern int tar_speed2;
extern int cur_speed2;
extern int tar_acc2;
extern int cur_acc2;

//电机3参数
extern int tar_pulse3;
extern int cur_pulse3;
extern int tar_speed3;
extern int cur_speed3;
extern int tar_acc3;
extern int cur_acc3;

//电机4参数
extern int tar_pulse4;
extern int cur_pulse4;
extern int tar_speed4;
extern int cur_speed4;
extern int tar_acc4;
extern int cur_acc4;

//电机5参数
extern int tar_pulse5;
extern int cur_pulse5;
extern int tar_speed5;
extern int cur_speed5;
extern int tar_acc5;
extern int cur_acc5;

//电机6参数
extern int tar_pulse6;
extern int cur_pulse6;
extern int tar_speed6;
extern int cur_speed6;
extern int tar_acc6;
extern int cur_acc6;


extern int foc_cut[6];
extern char key_flag;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
