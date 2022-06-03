/*
 * bluetooth.h
 *
 *  Created on: 17/03/2022
 *      Author: LEO
 */

#ifndef INC_BLUETOOTH_H_
#define INC_BLUETOOTH_H_

#include "cmsis_os.h"
#include "stm32f7xx_hal.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include <stdlib.h>

#define  DEFAULT_TIMEOUT  20;
#define  DEBUG_KEYA 1

typedef  struct {
	UART_HandleTypeDef *huart_blu;
    osMutexId_t *buff_TXmutex;
	osMutexId_t *buff_RXmutex;
	osSemaphoreId_t *buff_TXsem;
	osSemaphoreId_t *buff_RXsem;
	int BUFF_SIZE;
	int buff_len;
	uint8_t buff_reading[50];
}buff_t;


void write_bluetooth(buff_t *device_blu,uint8_t *data_blu, int len);

bool read_bluetooth();

#endif /* INC_BLUETOOTH_H_ */
