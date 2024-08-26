/*
 * onewire.c
 *
 *  Created on: 13.02.2012
 *      Author: di
 */
//#include "stm32l1xx_hal_uart.h"
#include "stm32F4xx_hal.h"
#include "stm32F4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "onewire.h"
#include "cmsis_os.h"

extern void xPortSysTickHandler(void);
extern  DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
//extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;


#ifdef OW_USART1

#undef OW_USART2
#undef OW_USART3
#undef OW_USART4

#define OW_USART 		USART1
#define OW_DMA_CH_RX 	DMA1_Channel5
#define OW_DMA_CH_TX 	DMA1_Channel4
#define OW_DMA_FLAG		DMA1_FLAG_TC5

#endif


#ifdef OW_USART2

#undef OW_USART1
#undef OW_USART3
#undef OW_USART4

#define OW_USART 		USART2
#define OW_DMA_CH_RX 	DMA1_Channel6
#define OW_DMA_CH_TX 	DMA1_Channel7
#define OW_DMA_FLAG		DMA1_FLAG_TC6

#endif


// Буфер для приема/передачи по 1-wire
uint8_t ow_buf[8];

#define OW_0	0x00
#define OW_1	0xff
#define OW_R_1	0xff

//-----------------------------------------------------------------------------
// функция преобразует один байт в восемь, для передачи через USART
// ow_byte - байт, который надо преобразовать
// ow_bits - ссылка на буфер, размером не менее 8 байт
//-----------------------------------------------------------------------------
void OW_toBits(uint8_t ow_byte, uint8_t *ow_bits) {
	uint8_t i;
	for (i = 0; i < 8; i++) {
		if (ow_byte & 0x01) {
			*ow_bits = OW_1;
		} else {
			*ow_bits = OW_0;
		}
		ow_bits++;
		ow_byte = ow_byte >> 1;
	}
}

//-----------------------------------------------------------------------------
// обратное преобразование - из того, что получено через USART опять собирается байт
// ow_bits - ссылка на буфер, размером не менее 8 байт
//-----------------------------------------------------------------------------
uint8_t OW_toByte(uint8_t *ow_bits) {
	uint8_t ow_byte, i;
	ow_byte = 0;
	for (i = 0; i < 8; i++) {
		ow_byte = ow_byte >> 1;
		if (*ow_bits == OW_R_1) {
			ow_byte |= 0x80;
		}
		ow_bits++;
	}

	return ow_byte;
}

//-----------------------------------------------------------------------------
// инициализирует USART и DMA
//-----------------------------------------------------------------------------
uint8_t OW_Init() {
	
  huart2.Instance = OW_USART;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);

	return OW_OK;
}
void OW_SendBits(uint8_t num_bits) {

	// Ждем, пока не примем 8 байт
HAL_UART_Receive_DMA(&huart2, ow_buf, num_bits);
HAL_UART_Transmit_DMA(&huart2, ow_buf, num_bits);
	while (HAL_DMA_STATE_READY != HAL_DMA_GetState(&hdma_usart2_rx))
{
#ifdef OW_GIVE_TICK_RTOS
	osDelay(1);	
#endif
	}
	HAL_UART_DMAStop(&huart2);
}
//-----------------------------------------------------------------------------
// осуществляет сброс и проверку на наличие устройств на шине
//-----------------------------------------------------------------------------
uint8_t OW_Reset() {
	uint8_t ow_presence = 0, tmp=0xf0;
	//tmp[0] =0xf0;

	
  huart2.Init.BaudRate = 9600;
  HAL_UART_Init(&huart2);

	// отправляем 0xf0 на скорости 9600

	HAL_UART_Transmit(&huart2, &tmp, 1, 100);

	while (HAL_UART_STATE_BUSY_TX == HAL_UART_GetState(&huart2)) {
#ifdef OW_GIVE_TICK_RTOS
		osDelay(1);
#endif
}

	HAL_UART_Receive(&huart2, &ow_presence, 1, 100);

  huart2.Init.BaudRate = 115200;
  HAL_UART_Init(&huart2);

	if (ow_presence != 0xf0) {
		return OW_OK;
	}

	return OW_NO_DEVICE;
}


//-----------------------------------------------------------------------------
// процедура общения с шиной 1-wire
// sendReset - посылать RESET в начале общения.
// 		OW_SEND_RESET или OW_NO_RESET
// command - массив байт, отсылаемых в шину. Если нужно чтение - отправляем OW_READ_SLOTH
// cLen - длина буфера команд, столько байт отошлется в шину
// data - если требуется чтение, то ссылка на буфер для чтения
// dLen - длина буфера для чтения. Прочитается не более этой длины
// readStart - с какого символа передачи начинать чтение (нумеруются с 0)
//		можно указать OW_NO_READ, тогда можно не задавать data и dLen
//-----------------------------------------------------------------------------
uint8_t OW_Send(uint8_t sendReset, uint8_t *command, uint8_t cLen,
		uint8_t *data, uint8_t dLen, uint8_t readStart) {

	// если требуется сброс - сбрасываем и проверяем на наличие устройств
	if (sendReset == OW_SEND_RESET) {
		if (OW_Reset() == OW_NO_DEVICE) {
			return OW_NO_DEVICE;
		}
	}

	while (cLen > 0) {
//		DMA_InitTypeDef DMA_InitStructure;

		OW_toBits(*command, ow_buf);
		command++;
		cLen--;

HAL_UART_Receive_DMA(&huart2, ow_buf, 8);
HAL_UART_Transmit_DMA(&huart2, ow_buf, 8);
	while (HAL_DMA_STATE_READY != HAL_DMA_GetState(&hdma_usart2_rx))
{
#ifdef OW_GIVE_TICK_RTOS
	osDelay(1);	
#endif
}

	HAL_UART_DMAStop(&huart2);

		// если прочитанные данные кому-то нужны - выкинем их в буфер
		if (readStart == 0 && dLen > 0) {
			*data = OW_toByte(ow_buf);
			data++;
			dLen--;
		} else {
			if (readStart != OW_NO_READ) {
				readStart--;
			}
		}
	}

	return OW_OK;
}
//-----------------------------------------------------------------------------
// Данная функция осуществляет сканирование сети 1-wire и записывает найденные
//   ID устройств в массив buf, по 8 байт на каждое устройство.
// переменная num ограничивает количество находимых устройств, чтобы не переполнить
// буфер.
//-----------------------------------------------------------------------------
uint8_t OW_Scan(uint8_t *buf, uint8_t num) {

	uint8_t found = 0;
	uint8_t *lastDevice;
	uint8_t *curDevice = buf;
	uint8_t numBit, lastCollision, currentCollision, currentSelection;

	lastCollision = 0;
	while (found < num) {
		numBit = 1;
		currentCollision = 0;

		// посылаем команду на поиск устройств
		OW_Send(OW_SEND_RESET, (uint8_t*)"\xf0", 1, 0, 0, OW_NO_READ);

		for (numBit = 1; numBit <= 64; numBit++) {
			// читаем два бита. Основной и комплементарный
			OW_toBits(OW_READ_SLOT, ow_buf);
			OW_SendBits(2);

			if (ow_buf[0] == OW_R_1) {
				if (ow_buf[1] == OW_R_1) {
					// две единицы, где-то провтыкали и заканчиваем поиск
					return found;
				} else {
					// 10 - на данном этапе только 1
					currentSelection = 1;
				}
			} else {
				if (ow_buf[1] == OW_R_1) {
					// 01 - на данном этапе только 0
					currentSelection = 0;
				} else {
					// 00 - коллизия
					if (numBit < lastCollision) {
						// идем по дереву, не дошли до развилки
						if (lastDevice[(numBit - 1) >> 3]
								& 1 << ((numBit - 1) & 0x07)) {
							// (numBit-1)>>3 - номер байта
							// (numBit-1)&0x07 - номер бита в байте
							currentSelection = 1;

							// если пошли по правой ветке, запоминаем номер бита
							if (currentCollision < numBit) {
								currentCollision = numBit;
							}
						} else {
							currentSelection = 0;
						}
					} else {
						if (numBit == lastCollision) {
							currentSelection = 0;
						} else {
							// идем по правой ветке
							currentSelection = 1;

							// если пошли по правой ветке, запоминаем номер бита
							if (currentCollision < numBit) {
								currentCollision = numBit;
							}
						}
					}
				}
			}

			if (currentSelection == 1) {
				curDevice[(numBit - 1) >> 3] |= 1 << ((numBit - 1) & 0x07);
				OW_toBits(0x01, ow_buf);
			} else {
				curDevice[(numBit - 1) >> 3] &= ~(1 << ((numBit - 1) & 0x07));
				OW_toBits(0x00, ow_buf);
			}
			OW_SendBits(1);
		}
		found++;
		lastDevice = curDevice;
		curDevice += 8;
		if (currentCollision == 0)
			return found;

		lastCollision = currentCollision;
	}

	return found;
}
//-----------------------------------------------------------------------------
// Данная функция осуществляет чтение ds18b20 по ID
//   ID устройста 8 байт на каждое устройство.
// 	возврощает прочитанные значения
//-----------------------------------------------------------------------------

Type_sensor OW_Read(char *id) {
	
uint8_t buf[2]= {0};
Type_sensor temp_S = {0};
		//чтение первого устройства
	OW_Send(OW_SEND_RESET, (uint8_t*)"\x55", 1, NULL, NULL, OW_NO_READ);
	OW_Send(OW_NO_RESET, (uint8_t*)id, 8, NULL, NULL, OW_NO_READ);
	OW_Send(OW_NO_RESET,(uint8_t*)"\x44" , 1, NULL, NULL, OW_NO_READ);
       vTaskDelay(800); // Выдержка 230 мс
	OW_Send(OW_SEND_RESET, (uint8_t*)"\x55", 1, NULL, NULL, OW_NO_READ);
	OW_Send(OW_NO_RESET, (uint8_t*)id, 8, NULL, NULL, OW_NO_READ);		
	OW_Send(OW_NO_RESET, (uint8_t*)"\xbe\xff\xff", 3, buf,2, 1);
		if (buf[1] >= 0x80)
		{
			temp_S.deg = buf[1];
			temp_S.deg <<= 8;
			temp_S.deg |= buf[0];
			temp_S.deg = (~temp_S.deg)+1;
			temp_S.deg = (temp_S.deg *10)/16;
			temp_S.znak = '-';
		}
		else
		{
		temp_S.deg = buf[1];
		temp_S.deg <<= 8;
		temp_S.deg |= buf[0];
		temp_S.deg = (temp_S.deg *10)/16;
		temp_S.znak = '+';

		}
		
		return temp_S;
	}

Type_sensor OW_Read_s() {

uint8_t buf[2]= {0};
Type_sensor temp_S = {0};
		//чтение первого устройства
/*
	OW_Send(OW_SEND_RESET, (uint8_t*)"\x55", 1, NULL, NULL, OW_NO_READ);
	OW_Send(OW_NO_RESET, (uint8_t*)id, 8, NULL, NULL, OW_NO_READ);
	OW_Send(OW_NO_RESET,(uint8_t*)"\x44" , 1, NULL, NULL, OW_NO_READ);
       vTaskDelay(800); // Выдержка 230 мс
	OW_Send(OW_SEND_RESET, (uint8_t*)"\x55", 1, NULL, NULL, OW_NO_READ);
	OW_Send(OW_NO_RESET, (uint8_t*)id, 8, NULL, NULL, OW_NO_READ);
	OW_Send(OW_NO_RESET, (uint8_t*)"\xbe\xff\xff", 3, buf,2, 1);*/
		if (buf[1] >= 0x80)
		{
			temp_S.deg = buf[1];
			temp_S.deg <<= 8;
			temp_S.deg |= buf[0];
			temp_S.deg = (~temp_S.deg)+1;
			temp_S.deg = (temp_S.deg *10)/16;
			temp_S.znak = '-';
		}
		else
		{
		temp_S.deg = buf[1];
		temp_S.deg <<= 8;
		temp_S.deg |= buf[0];
		temp_S.deg = (temp_S.deg *10)/16;
		temp_S.znak = '+';

		}

		return temp_S;
	}
