/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "eeprom.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//#define ow_uart huart1
//#define OW_USART USART1

extern TIM_HandleTypeDef htim13;

extern TIM_HandleTypeDef htim14;

extern UART_HandleTypeDef huart1;

float Temp[MAXDEVICES_ON_THE_BUS];
float current_temp;
extern trmo_settings termo_set;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t flag_pulse_Start = 0;

//float Temperature = 0.0;
uint8_t Presence = 0;
uint8_t Temp_byte1, Temp_byte2;
uint8_t one_wire_buff[9] = {0};
uint16_t TEMP;
//extern float setTEMP;
//float stepTemp = 3.0;
//extern float maxTemp;

uint32_t oldTime = 0;
uint32_t time = 0;

uint32_t time_DS18B20 = 0;
uint32_t time_out_DS18B20 = 1;
uint32_t time_DS18B20_Read = 0;
uint8_t event_DS18B20 = 0;

uint32_t time_Dimmer = 0;
//uint32_t time_Dimmer_Pulse = 0;
uint32_t time_out_dimmer = 20; // 20 mS не включается семистор 0 включен постоянно
uint8_t event_dimmer = 0;
uint32_t delay_dimm_us = 11000; // 10000 us = 10 ms

//stm log
uint8_t message_rx[message_RX_LENGTH];
uint8_t UART2_rx[UART2_RX_LENGTH];
uint16_t indx_message_rx = 0;
uint16_t indx_UART2_rx = 0;
uint16_t Size_message = 0;
uint16_t Start_index = 0;

/* USER CODE END Variables */
osThreadId MainTaskHandle;
osThreadId SettingsTaskHandle;
osMessageQId rxDataUART2Handle;
osSemaphoreId eepromSemHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void actoin_motor_set(cJSON *obj, uint8_t save);
void actoin_set(cJSON *obj, uint8_t save);
void actoin_resp_all_set(void);
void actoin_resp_status(void);

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static uint8_t DS18B20_Init(void);
static uint8_t DS18B20_ReadBit(void);
static uint8_t DS18B20_ReadByte(void);
static void DS18B20_WriteByte(uint8_t);
void DS18B20_SampleTemp(void);
float DS18B20_ReadTemp(void);
void DS18B20_usart_setup(uint32_t baud);
/* USER CODE END FunctionPrototypes */

void mainTask(void const * argument);
void settingsTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
		StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize) {
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
	*ppxIdleTaskStackBuffer = &xIdleStack[0];
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
	/* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of eepromSem */
  osSemaphoreDef(eepromSem);
  eepromSemHandle = osSemaphoreCreate(osSemaphore(eepromSem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of rxDataUART2 */
  osMessageQDef(rxDataUART2, 16, uint16_t);
  rxDataUART2Handle = osMessageCreate(osMessageQ(rxDataUART2), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of MainTask */
  osThreadDef(MainTask, mainTask, osPriorityNormal, 0, 512);
  MainTaskHandle = osThreadCreate(osThread(MainTask), NULL);

  /* definition and creation of SettingsTask */
  osThreadDef(SettingsTask, settingsTask, osPriorityNormal, 0, 512);
  SettingsTaskHandle = osThreadCreate(osThread(SettingsTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_mainTask */
/**
 * @brief  Function implementing the MainTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_mainTask */
void mainTask(void const * argument)
{
  /* USER CODE BEGIN mainTask */


	oldTime = HAL_GetTick();
	HAL_GPIO_WritePin(CTR_GPIO_Port, CTR_Pin, GPIO_PIN_RESET);
	HAL_TIM_Base_Start(&htim13);

	//get_ROMid();
	/* Infinite loop */
	for (;;) {
		// чтение температуры по таймауту
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		//get_Temperature();
		  DS18B20_SampleTemp();               // Convert (Sample) Temperature Now
		  osDelay(300);
		  Temp[0] = DS18B20_ReadTemp();  // Read The Conversion Result Temperature Value
		  current_temp = Temp[0];
		if (Temp[0] < termo_set.setTEMP) {
			//Temperature = 0;
			delay_dimm_us = 11000;
		} else if (Temp[0] > termo_set.maxTemp) {
			Temp[0] = termo_set.maxTemp;
			delay_dimm_us = 1;
		} else {
			// управление симистором
			delay_dimm_us = (uint32_t) map(Temp[0], termo_set.setTEMP, termo_set.maxTemp, FAN_START, 1);
		}
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		osDelay(700);

	}
  /* USER CODE END mainTask */
}

/* USER CODE BEGIN Header_settingsTask */
/**
 * @brief Function implementing the SettingsTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_settingsTask */
void settingsTask(void const * argument)
{
  /* USER CODE BEGIN settingsTask */
	//HAL_UART_Receive_DMA(&huart2, UART2_rx, UART2_RX_LENGTH);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, UART2_rx, UART2_RX_LENGTH);
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	//__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_TC);
	/* Infinite loop */
	for (;;) {
		// ожидать собщение
		osMessageGet(rxDataUART2Handle, osWaitForever);
		//uint32_t message_len = strlen((char*) message_rx);
		//HAL_UART_Transmit(&huart2, message_rx, message_len, HAL_MAX_DELAY);

		// парсим  json
		cJSON *json = cJSON_Parse((char*) message_rx);
		if (json != NULL) {

			cJSON *id = cJSON_GetObjectItemCaseSensitive(json, "id");
			cJSON *name_device = cJSON_GetObjectItemCaseSensitive(json, "name_device");
			cJSON *type_data = cJSON_GetObjectItemCaseSensitive(json, "type_data");
			cJSON *save_settings = cJSON_GetObjectItemCaseSensitive(json, "save_settings");
			cJSON *obj = cJSON_GetObjectItemCaseSensitive(json, "obj");

			if (cJSON_IsNumber(id) && cJSON_GetNumberValue(id) == ID_CTRL) {
				uint8_t save_set = 0;
				if (cJSON_IsTrue(save_settings)) {
					save_set = 1;
				} else {
					save_set = 0;
				}

				if (cJSON_IsNumber(type_data)) {
					switch (type_data->valueint) {
					case 1: // ip settings
						actoin_set(obj, save_set);
						break;
					case 2: // motor settings
						actoin_motor_set(obj, save_set);
						break;
					case 3:
						actoin_resp_all_set();
						break;
					case 4:
						actoin_resp_status();
						break;
					default:
						STM_LOG("data type not registered");
						break;
					}
				}
			} else {
				STM_LOG("id not valid");
			}

			cJSON_Delete(json);
		} else {
			STM_LOG("Invalid JSON");
		}

		osDelay(1);
	}
  /* USER CODE END settingsTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == Zc_INT_Pin) {
		HAL_GPIO_TogglePin(B2_GPIO_Port, B2_Pin);
		HAL_TIM_Base_Stop_IT(&htim14);
		__HAL_TIM_SET_AUTORELOAD(&htim14, delay_dimm_us); // таймар на открытие симистора
		__HAL_TIM_SET_COUNTER(&htim14, 0);
		HAL_GPIO_WritePin(CTR_GPIO_Port, CTR_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, GPIO_PIN_RESET);
		flag_pulse_Start = 0;
		HAL_TIM_Base_Start_IT(&htim14); // запуск таймара на открытие симистора
	} else {
		__NOP();
	}
}

void actoin_motor_set(cJSON *obj, uint8_t save) {


}

void actoin_set(cJSON *obj, uint8_t save) {

	cJSON *j_SET_TEMP = cJSON_GetObjectItemCaseSensitive(obj, "set_temp");
	cJSON *j_MAX_TEMP = cJSON_GetObjectItemCaseSensitive(obj, "max_temp");
	cJSON *j_STEP_TEMP = cJSON_GetObjectItemCaseSensitive(obj, "step_temp");

	if((j_SET_TEMP != NULL) && cJSON_IsNumber(j_SET_TEMP))
	{
		termo_set.setTEMP = (float)cJSON_GetNumberValue(j_SET_TEMP);
	}
	if((j_MAX_TEMP != NULL) && cJSON_IsNumber(j_MAX_TEMP))
	{
		termo_set.maxTemp = (float)cJSON_GetNumberValue(j_MAX_TEMP);
	}
	if((j_STEP_TEMP != NULL) && cJSON_IsNumber(j_STEP_TEMP))
	{
		termo_set.stepTemp = (float)cJSON_GetNumberValue(j_STEP_TEMP);
	}

	HAL_StatusTypeDef ret;
	HAL_FLASH_Unlock();
	ret = eeprom_write(termo_set);
	if (ret != HAL_OK) {
		STM_LOG("Fail write settings");
	}
	HAL_FLASH_Lock();
}

void actoin_resp_all_set() {

}

void actoin_resp_status() {
	//STM_LOG("actoin_resp_status()");
	cJSON *j_all_settings_obj = cJSON_CreateObject();
	cJSON *j_to_host = cJSON_CreateObject();

	cJSON_AddNumberToObject(j_to_host, "id", ID_CTRL);
	cJSON_AddStringToObject(j_to_host, "name_device", NAME);
	cJSON_AddNumberToObject(j_to_host, "type_data", 4);

	cJSON_AddNumberToObject(j_all_settings_obj, "set_temp", (double)termo_set.setTEMP);
	cJSON_AddNumberToObject(j_all_settings_obj, "max_temp", (double)termo_set.maxTemp);
	cJSON_AddNumberToObject(j_all_settings_obj, "step_temp", (double)termo_set.stepTemp);
	cJSON_AddNumberToObject(j_all_settings_obj, "current", (double)(current_temp));

	cJSON_AddItemToObject(j_to_host, "obj", j_all_settings_obj);

	char *str_to_host = cJSON_Print(j_to_host);
	//string out = str_to_host;
	//out += '\r';

	//HAL_UART_Transmit(&huart2, (uint8_t*)out.c_str(), out.size(), HAL_MAX_DELAY);
	STM_LOG("%s", str_to_host);

	cJSON_free(str_to_host);
	cJSON_Delete(j_to_host);
}

/******************************************************************************************************
 Handlers
 ******************************************************************************************************/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart->Instance == USART2) {

		while ( __HAL_UART_GET_FLAG(huart, UART_FLAG_TC) != SET) {
		};

		uint16_t Size_Data = Size - Start_index;

		HAL_UART_RxEventTypeTypeDef rxEventType;
		rxEventType = HAL_UARTEx_GetRxEventType(huart);
		switch (rxEventType) {
		case HAL_UART_RXEVENT_IDLE:
			//STM_LOG( "IDLE. Size:%d sd:%d sti:%d", Size, Size_Data, Start_index);
			// копировать с индекса сообщения
			memcpy(&message_rx[indx_message_rx], &UART2_rx[Start_index],
					Size_Data);

			//|| (message_rx[indx_message_rx + Size_Data - 1] == '\n')
			if ((message_rx[indx_message_rx + Size_Data - 1] == '\r')
					|| (message_rx[indx_message_rx + Size_Data - 1] == 0)) {
				message_rx[indx_message_rx + Size_Data] = 0;
				// выдать сигнал
				osMessagePut(rxDataUART2Handle, (uint32_t) indx_message_rx, 0);
				Size_message = 0;
				// обнулить индекс сообщения
				indx_message_rx = 0;
			} else {
				indx_message_rx += Size_Data;
			}

			Start_index = Size;

			//STM_LOG( "\n" );
			break;

		case HAL_UART_RXEVENT_HT:
			//STM_LOG( "HT Size:%d sd:%d sti:%d", Size, Size_Data, Start_index);
			break;

		case HAL_UART_RXEVENT_TC:
			//STM_LOG( "TC Size:%d sd:%d sti:%d", Size, Size_Data, Start_index);
			// скопировать в начало буфера
			memcpy(&message_rx[indx_message_rx], &UART2_rx[Start_index],
					Size_Data);
			// сохронить индекс сообщения
			indx_message_rx += Size_Data;
			Start_index = 0;
			break;

		default:
			STM_LOG("???");
			break;
		}

		HAL_UARTEx_ReceiveToIdle_DMA(huart, UART2_rx, UART2_RX_LENGTH);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
		//usart_rx_check(Size);
	}

}

static uint8_t DS18B20_Init(void)
{
	uint8_t ResetByte = 0xF0, PresenceByte;
	//LL_USART_SetBaudRate(huart1.Instance, HAL_RCC_GetPCLK2Freq(), 9600);
	DS18B20_usart_setup(9600);
	// Send reset pulse (0xF0)
	HAL_UART_Transmit(&huart1, &ResetByte, 1, 1);
	// Wait for the presence pulse
	HAL_UART_Receive(&huart1, &PresenceByte, 1, 1);
	//LL_USART_SetBaudRate(huart1.Instance, HAL_RCC_GetPCLK2Freq(), 115200);
	DS18B20_usart_setup(115200);
	// Check presence pulse
	if (PresenceByte != ResetByte){
		return 1; // Presence pulse detected
	}
	else{
		return 0; // No presence pulse detected
	}
}

static uint8_t DS18B20_ReadBit(void)
{
    uint8_t ReadBitCMD = 0xFF;
    uint8_t RxBit;

    // Send Read Bit CMD
    HAL_UART_Transmit(&huart1, &ReadBitCMD, 1, 1);
    // Receive The Bit
    //HAL_UART_Receive(&huart1, &RxBit, 1, 1);
    RxBit = (uint8_t)(huart1.Instance->DR & (uint8_t)0x00FF);

    return (RxBit & 0x01);
}

static uint8_t DS18B20_ReadByte(void)
{
	uint8_t RxByte = 0;
	for (uint8_t i = 0; i < 8; i++)
	{
		RxByte >>= 1;
		if (DS18B20_ReadBit())
		{
			RxByte |= 0x80;
		}
	}
	return RxByte;
}

static void DS18B20_WriteByte(uint8_t data)
{
	uint8_t TxBuffer[8];
    for (int i=0; i<8; i++)
    {
	  if ((data & (1<<i)) != 0){
		  TxBuffer[i] = 0xFF;
	  }
	  else{
		  TxBuffer[i] = 0;
	  }
    }
    HAL_UART_Transmit(&huart1, TxBuffer, 8, 10);
}

void DS18B20_SampleTemp(void)
{
	DS18B20_Init();
	DS18B20_WriteByte(0xCC);  // Skip ROM   (ROM-CMD)
	DS18B20_WriteByte(0x44);  // Convert T  (F-CMD)
}

float DS18B20_ReadTemp(void)
{
	uint8_t Temp_LSB, Temp_MSB;
	uint16_t Temp;
	float Temperature;

	DS18B20_Init();
	DS18B20_WriteByte(0xCC);  // Skip ROM         (ROM-CMD)
	DS18B20_WriteByte(0xBE);  // Read Scratchpad  (F-CMD)
	Temp_LSB = DS18B20_ReadByte();
	Temp_MSB = DS18B20_ReadByte();
	Temp = ((Temp_MSB<<8))|Temp_LSB;
	Temperature = (float)Temp/16.0;

	return Temperature;
}

void DS18B20_usart_setup(uint32_t baud) {

	huart1.Instance = USART1;
	huart1.Init.BaudRate = baud;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;

	HAL_UART_Abort_IT(&huart1);
	HAL_UART_DeInit(&huart1);
	if (HAL_UART_Init(&huart1) != HAL_OK) //HAL_UART_Init
	{
		//	    Error_Handler();
		__asm__("NOP");
	}

	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}
/* USER CODE END Application */
