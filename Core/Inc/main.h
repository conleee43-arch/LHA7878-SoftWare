/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

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
#define LED_Pin GPIO_PIN_3
#define LED_GPIO_Port GPIOE
#define DAISY_IN_Pin GPIO_PIN_6
#define DAISY_IN_GPIO_Port GPIOA
#define DRDY__Pin GPIO_PIN_0
#define DRDY__GPIO_Port GPIOB
#define START_Pin GPIO_PIN_10
#define START_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_8
#define SPI2_CS_GPIO_Port GPIOC
#define PWDN__Pin GPIO_PIN_6
#define PWDN__GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* LHA7878 驱动引脚映射 (适配 CubeMX 生成的标签) */
#define LHA7878_SPI2_CS_GPIO_Port   SPI2_CS_GPIO_Port
#define LHA7878_SPI2_CS_Pin         SPI2_CS_Pin
#define LHA7878_START_GPIO_Port     START_GPIO_Port
#define LHA7878_START_Pin           START_Pin
#define LHA7878_nPWDN_GPIO_Port     PWDN__GPIO_Port
#define LHA7878_nPWDN_Pin           PWDN__Pin
#define LHA7878_DRDY_GPIO_Port      DRDY__GPIO_Port
#define LHA7878_DRDY_Pin            DRDY__Pin

/* 注意：当前工程 CubeMX 未分配 RESET 引脚。
 * 依据硬件 Demoboard 原理图及参考驱动工程 (v0.10)，RESET 默认连接至 PB7。
 * 若板级硬件 RESET 已直接外部上拉至 DVDD，驱动亦可通过软件 RESET 命令工作。 */
#define LHA7878_nRESET_GPIO_Port    GPIOB
#define LHA7878_nRESET_Pin          GPIO_PIN_7
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
