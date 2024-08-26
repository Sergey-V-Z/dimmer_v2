/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdarg.h>
#include <stdio.h>
#include "eeprom.h"
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

/* USER CODE BEGIN PV */
//float setTEMP = SETTEMP;
//float stepTemp = 3.0;
//float maxTemp = MAXTEMP;
trmo_settings termo_set;
uint16_t VirtAddVarTab[NB_OF_VAR];
uint16_t VarDataTab[NB_OF_VAR];

uint16_t VarDataTabRead[NB_OF_VAR];
uint16_t VarIndex, VarDataTmp = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
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
  MX_DMA_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	STM_LOG("start dimmer");

	/* Unlock the Flash Program Erase controller */
	HAL_FLASH_Unlock();
	/* EEPROM Init */
	STM_LOG("eeprom init start");
	if (EE_Init() != EE_OK) {
		STM_LOG("eeprom init fault");
	} else {
		// Fill EEPROM variables addresses
		for (VarIndex = 1; VarIndex <= NB_OF_VAR; VarIndex++) {
			VirtAddVarTab[VarIndex - 1] = VarIndex;
		}

		HAL_StatusTypeDef ret;
		ret = eeprom_read(&termo_set);
		if (ret != HAL_OK) {
			STM_LOG("var  not foud in eeprom. wtite default");
			termo_set.setTEMP = SETTEMP;
			termo_set.maxTemp = MAXTEMP;
			termo_set.stepTemp = STEPTEMP;

			ret = eeprom_write(termo_set);
			if (ret != HAL_OK) {
				eeprom_read(&termo_set);
			}
			termo_set.setTEMP = 0;
			termo_set.maxTemp = 0;
			termo_set.stepTemp = 0;
			eeprom_read(&termo_set);

		}

	}
	HAL_FLASH_Lock();
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

/* USER CODE BEGIN 4 */
uint8_t log_tx_buffer[LOG_TX_BUF_SIZE + 2];

void STM_LOG(const char *format, ...) {
	while (DBG_PORT.gState != HAL_UART_STATE_READY);

	va_list args;
	int size = 0;

	va_start(args, format);
	//vsprintf((char *)log_tx_buffer, format, args);
	size = vsnprintf((char*) log_tx_buffer, LOG_TX_BUF_SIZE, format, args);
	va_end(args);

	// добавить \r
	log_tx_buffer[size] = '\r';
	log_tx_buffer[size + 1] = 0;

	HAL_UART_Transmit_DMA(&DBG_PORT, log_tx_buffer,
			strlen((const char*) log_tx_buffer));
}

uint16_t eeprom_read(trmo_settings *ts) {
	uint16_t ret;
	/*uint16_t arr[4];

	 for (VarIndex = 0; VarIndex < NB_OF_VAR; VarIndex++)
	 {
	 if((EE_ReadVariable(VirtAddVarTab[VarIndex],  &VarDataTabRead[VarIndex])) != HAL_OK)
	 {
	 Error_Handler();
	 }
	 }

	 *Data = (*(float*) &arr[start_vIndex]);*/

	for (VarIndex = 0; VarIndex < NB_OF_VAR; VarIndex++) {
		ret = EE_ReadVariable(VirtAddVarTab[VarIndex], &VarDataTabRead[VarIndex]);
		if (ret != HAL_OK) {
			STM_LOG("fail write eeprom: vindex %d", VarIndex);
			return ret;
		}
	}

	*ts = (*(trmo_settings*) VarDataTabRead);
	return ret;
}

uint16_t eeprom_write(trmo_settings ts) {
	uint16_t ret;
	/*uint16_t *arr;
	 arr = (uint16_t*)(&inData);

	 for (int VarIndex = start_vIndex; VarIndex < 2; VarIndex++) {
	 ret = EE_ReadVariable(VarIndex, &arr[VarIndex]);
	 if (ret != HAL_OK) {
	 STM_LOG("fail write eeprom: vindex %d", VarIndex);
	 return ret;
	 }
	 }*/
	//uint16_t *arr = (uint16_t*)(&ts);
	//VarDataTabRead = (uint16_t*)(&ts);
	memcpy(VarDataTabRead, (uint16_t*)(&ts), sizeof(ts));

	for (VarIndex = 0; VarIndex < NB_OF_VAR; VarIndex++) {
		ret = EE_WriteVariable(VirtAddVarTab[VarIndex], VarDataTabRead[VarIndex]);
		if (ret != HAL_OK) {
			STM_LOG("fail write eeprom: vindex %d", VarIndex);
			return ret;
		}
	}
	return ret;
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM12 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM12) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	if (htim->Instance == htim14.Instance) {
		HAL_TIM_Base_Stop_IT(&htim14);
		HAL_GPIO_WritePin(CTR_GPIO_Port, CTR_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
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
	__disable_irq();
	while (1) {
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
