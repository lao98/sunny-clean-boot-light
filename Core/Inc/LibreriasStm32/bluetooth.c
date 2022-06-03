/*
 * bluetooth.c
 *
 *  Created on: 17/03/2022
 *      Author: PC
 */

#include "include/bluetooth.h"

extern DMA_HandleTypeDef hdma_usart6_rx;


void write_bluetooth(buff_t *device_blu,uint8_t *data_blu, int len)
{

	osSemaphoreAcquire(*device_blu->buff_TXsem,osWaitForever);
	HAL_UART_Transmit_IT(device_blu->huart_blu, data_blu, len);
}

bool read_bluetooth(buff_t *device_blu)
{
    osMutexAcquire(*device_blu->buff_RXmutex, osWaitForever);
	bool status;
	//receive the data using dma (direct memory access) to avoid excessive consumption
	//puntero -> atributo
	memset(device_blu->buff_reading,0,50);
	status=HAL_UART_Receive_DMA(device_blu->huart_blu,device_blu->buff_reading, 50)==HAL_OK ? 1:0;
	osSemaphoreAcquire(*device_blu->buff_RXsem,  (TickType_t)10*portTICK_PERIOD_MS);
	osStatus_t semStatus=osSemaphoreAcquire(*device_blu->buff_RXsem, osWaitForever);// (TickType_t)10*portTICK_PERIOD_MS);
	if( semStatus!=osOK || status==false){
		status=false;
	}
	HAL_UART_DMAStop(device_blu->huart_blu);
	/*if (*device_blu->buff_reading = "1") {

	    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
	    HAL_Delay(500);
    }
    */
#if DEBUG_KEYA
	printf("R:%s\n",(char *)device_blu->buff_reading);
#endif
	osSemaphoreRelease(*device_blu->buff_RXsem);
	osMutexRelease(*device_blu->buff_RXmutex);
	return status;

}

