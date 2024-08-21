/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cJSON.h"
#include <stdarg.h>
#include <string.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

#define DBG_PORT huart2
#define LOG_TX_BUF_SIZE 2048
#define UART2_RX_LENGTH 512
#define message_RX_LENGTH 512

#define CURENT_VERSION 46
#define ID_CTRL 3
#define NAME "dimmer"

#define ow_uart huart1
#define OW_USART USART1
#ifndef MAXDEVICES_ON_THE_BUS
#define MAXDEVICES_ON_THE_BUS 1  // maximum planned number of devices on the bus
#endif

#define TIME_OUT_DS18B20_Read 1000
//#define TIME_OUT_DIMMER 1000
#define TIME_OUT_DIMMER_PULSE 1

#define PERIOD_US 10000		// 10 ms
#define DELAY_PULSE_US 5000 // 5 ms
#define FAN_0 10000		// 0 %
#define FAN_P1 5000		// 50 %
#define FAN_P2 4000		// 60 %
#define FAN_P3 3000		// 70 %
#define FAN_P4 2000		// 80 %
#define FAN_P5 5		// 100 %

#define FAN_START 7000 // 30 %
#define SETTEMP 30
#define STEPTEMP 15
#define MAXTEMP (SETTEMP + STEPTEMP)

typedef struct {
	float setTEMP;
	float maxTemp;
	float stepTemp;
}trmo_settings;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void STM_LOG(const char* format, ...);
uint16_t eeprom_read(trmo_settings *ts);
uint16_t eeprom_write(trmo_settings ts);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_6
#define B1_GPIO_Port GPIOA
#define B2_Pin GPIO_PIN_7
#define B2_GPIO_Port GPIOA
#define CTR_Pin GPIO_PIN_1
#define CTR_GPIO_Port GPIOB
#define Zc_INT_Pin GPIO_PIN_12
#define Zc_INT_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_7
#define LED_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
