/*
 * MDC100.c
 *
 *  Created on: Jun 18, 2021
 *      Author: jeison Estiven garcia
 */

#include "include/MDC100.h"


mdc100_status_t mdc100_begin(mdc100_t *device){
	return mdc100_writeCommand(device, MDC_100_START_MOTOR);
}

mdc100_status_t mdc100_checkError(mdc100_t *device){

	char command[5]={0,0,0,0,0};
	int len=sprintf(command,"@0%s\r",MDC_100_ERROR_CODE);
	bool status=mdc100_writeData(device,(uint8_t *)command, len);
	char buffer[10]={0,0,0,0,0,0,0,0,0,0};
	if(status){
		if(mdc100_readData(device,(uint8_t*) buffer)){
			uint8_t data_length  = 0;
			while(buffer[data_length]!=0){
				data_length++;
			}
			int value=0;
			if(mdc100_Str2Int((uint8_t *)buffer, data_length, &value) != MDC_100_OK) return MDC_100_CORRUPT_DATA;
			return value;
		}else{
			return MDC_100_TIMEOUT_RX;
		}
	}else{
		return MDC_100_TRANSMIT_ERROR;
	}
}

mdc100_status_t mdc100_readCommand(mdc100_t *device,const char* command, uint8_t* buffer){
	char cmd[20];
	int len=sprintf(cmd,"@0%s\r",command);
	if(mdc100_writeData(device,(uint8_t *) cmd, len)){
		if(mdc100_readData(device,buffer)){
			return mdc100_checkError(device);
		}else{
			return MDC_100_TIMEOUT_RX;
		}
	}else{
		return MDC_100_TRANSMIT_ERROR;
	}
}

mdc100_status_t mdc100_writeCommand(mdc100_t *device,const char* command){
	char buffer[20];
	int len=sprintf(buffer,"@0%s\r",command);
	if(mdc100_writeData(device,(uint8_t *) buffer, len)){
		return mdc100_checkError(device);
	}else{
		return MDC_100_TRANSMIT_ERROR;
	}
}

mdc100_status_t mdc100_writeCommand_param(mdc100_t *device,const char* command, int param, int low_limit, int upp_limit)
{
	int value=param;
	if(param>upp_limit){
		value=upp_limit;
	}
	if(param<low_limit){
		value=low_limit;
	}
	char buffer[20];
	int len=sprintf(buffer,"@0%s%d\r",command,value);
	if(mdc100_writeData(device,(uint8_t *) buffer, len)){
		if(strcmp(command,MDC_100_DIGITAL_SET_POINT)==0){
			osDelay(5);
		}
		return mdc100_checkError(device);
	}else{
		return MDC_100_TRANSMIT_ERROR;
	}
}

mdc100_status_t mdc100_Str2Int(uint8_t* buffer, int len, int* value){
	int result=0;
	int success=0;
	if(len<=0){
		return MDC_100_CORRUPT_DATA;
	}
	for (int i=len-1; i>=0; i--){
		if(buffer[len-1-i]>='0' && buffer[len-1-i]<='9'){
			result=result+(buffer[len-1-i]-'0')*pow(10,i-1);
			success++;
		}
	}
	if(success==len-1){
		*value=result;
		return MDC_100_OK;
	}else{
		return MDC_100_CORRUPT_DATA;
	}
}

mdc100_status_t mdc100_setMode(mdc100_t *device, char mode){
	switch(mode){
	case 'D':
		return  mdc100_writeCommand(device, MDC_100_DIGITAL_MODE);
	case 'A':
		return  mdc100_writeCommand(device, MDC_100_ANALOG_MODE);
	default:
		return MDC_100_INVALID_OPTION;
	}
}

mdc100_status_t mdc100_setDirection(mdc100_t *device, char direction){
	switch(direction){
		case '+':
			return  mdc100_writeCommand(device, MDC_100_COUNTER_CLOCKWISE_DIRECTION);
		case '-':
			return  mdc100_writeCommand(device, MDC_100_CLOCKWISE_DIRECTION);
		default:
			return MDC_100_INVALID_OPTION;
		}
}

mdc100_status_t mdc100_softBreak(mdc100_t *device){
	return mdc100_writeCommand(device, MDC_100_COAST);
}

mdc100_status_t mdc100_hardBreak(mdc100_t *device){
	return mdc100_writeCommand(device, MDC_100_HARD_BREAK);
}

mdc100_status_t mdc100_setSpeed(mdc100_t *device, int speed){
	mdc100_status_t status=mdc100_begin(device);
	if(status==MDC_100_OK){
		if(speed>0){
			return mdc100_writeCommand_param(device, MDC_100_DIGITAL_SET_POINT,speed, 10, 5500) |
				   mdc100_setDirection(device, '+');
		}else{
			if(speed<0){
				return mdc100_writeCommand_param(device, MDC_100_DIGITAL_SET_POINT,-1*speed, 10, 5500)|
					   mdc100_setDirection(device, '-') ;
			}
			return mdc100_softBreak(device);
		}
	}
	return status;
}

mdc100_status_t mdc100_analogLimits(mdc100_t *device,int lower_speed, int Upper_speed){
	if(Upper_speed<lower_speed){
		return MDC_100_INVALID_OPTION;
	}else{
		mdc100_status_t status=mdc100_writeCommand_param(device,MDC_100_ANALOG_SPEEDMIN_LIM,
				lower_speed, 0, 5500);
		if(status==MDC_100_OK){
			return mdc100_writeCommand_param(device,MDC_100_ANALOG_SPEEDMIN_LIM,
					lower_speed, 0, 5500);
		}else{
			return status;
		}
	}
}

mdc100_status_t mdc100_setPoles(mdc100_t *device, int poles){
	return mdc100_writeCommand_param(device, MDC_100_SET_NUMBER_POLES, poles, 2, 12);
}

mdc100_status_t mdc100_factoryReset(mdc100_t *device){
	return mdc100_writeCommand(device, MDC_100_FACTORY_DEFAULT);
}

mdc100_status_t mdc100_setKG(mdc100_t *device, int kg){
	return mdc100_writeCommand_param(device, MDC_100_SET_KG_VALUE, kg, 0, 254);
}

mdc100_status_t mdc100_setKP(mdc100_t *device, int kp){
	return mdc100_writeCommand_param(device, MDC_100_SET_KP_VALUE, kp, 0, 150);
}

mdc100_status_t mdc100_setKI(mdc100_t *device, int ki){
	return mdc100_writeCommand_param(device, MDC_100_SET_KI_VALUE, ki, 1, 9);
}

mdc100_status_t mdc100_setIV(mdc100_t *device, int iv){
	return mdc100_writeCommand_param(device, MDC_100_SET_INITIAL_VALUE, iv, 0, 1400);
}

mdc100_status_t mdc100_getSpeed(mdc100_t *device, int* value){
	uint8_t buffer[10]={0,0,0,0,0,0,0,0,0,0};
	mdc100_status_t status=mdc100_readCommand(device, MDC_100_READ_RPM_MOTOR, buffer);
	if(status==MDC_100_OK){
		uint8_t data_length  = 0;
		while(buffer[data_length]!=0){
			data_length++;
		}
		return mdc100_Str2Int(&buffer[1],data_length-1, value);
	}else{
		return status;
	}
}

mdc100_status_t mdc100_getSpeedCommandAnalog(mdc100_t *device, int* value){
	uint8_t buffer[10]={0,0,0,0,0,0,0,0,0,0};
	mdc100_status_t status=mdc100_readCommand(device, MDC_100_READ_SET_RPM, buffer);
	if(status==MDC_100_OK){
		uint8_t data_length  = 0;
		while(buffer[data_length]!=0){
			data_length++;
		}
		return mdc100_Str2Int(&buffer[1],data_length-1, value);
	}else{
		return status;
	}
}

mdc100_status_t mdc100_getLimitsAnalog(mdc100_t *device,int* value){
	uint8_t buffer[10]={0,0,0,0,0,0,0,0,0,0};
	mdc100_status_t status=mdc100_readCommand(device, MDC_100_READ_SPEEDMIN_ANALOG, buffer);
	if(status==MDC_100_OK){
		uint8_t data_length  = 0;
		while(buffer[data_length]!=0){
			data_length++;
		}
		mdc100_Str2Int(&buffer[1],data_length-1, &value[0]);
	}else{
		return status;
	}
	memset(buffer,0,10);
	status=mdc100_readCommand(device, MDC_100_READ_SPEEDMAX_ANALOG, buffer);
	if(status==MDC_100_OK){
		uint8_t data_length  = 0;
		while(buffer[data_length]!=0){
			data_length++;
		}
		return mdc100_Str2Int(&buffer[1],data_length-1, &value[1]);
	}else{
		return status;
	}
}

mdc100_status_t mdc100_getSpeedCommandDigital(mdc100_t *device, int* value){
	uint8_t buffer[10]={0,0,0,0,0,0,0,0,0,0};
	mdc100_status_t status=mdc100_readCommand(device, MDC_100_READ_DIGITAL_SET_RPM, buffer);
	if(status==MDC_100_OK){
		uint8_t data_length  = 0;
		while(buffer[data_length]!=0){
			data_length++;
		}
		return mdc100_Str2Int(&buffer[2],data_length-1, value);
	}else{
		return status;
	}
}

mdc100_status_t mdc100_getPoles(mdc100_t *device,int* value){
	uint8_t buffer[10]={0,0,0,0,0,0,0,0,0,0};
	mdc100_status_t status=mdc100_readCommand(device, MDC_100_READ_NUMBER_POLES, buffer);
	if(status==MDC_100_OK){
		uint8_t data_length  = 0;
		while(buffer[data_length]!=0){
			data_length++;
		}
		return mdc100_Str2Int(&buffer[1],data_length-1, value);
	}else{
		return status;
	}
}

mdc100_status_t mdc100_getDirection(mdc100_t *device,char* value){
	uint8_t buffer[10]={0,0,0,0,0,0,0,0,0,0};
	mdc100_status_t status=mdc100_readCommand(device, MDC_100_READ_DIRECTION_MOTOR, buffer);
	if(status==MDC_100_OK){
		if(buffer[1]=='+' || buffer[1]=='-'){
			 *value=(char) buffer[1];
			 return MDC_100_OK;
		}else{
			return MDC_100_CORRUPT_DATA;
		}
	}else{
		return status;
	}
}

mdc100_status_t mdc100_getMode(mdc100_t *device,char* value){
	uint8_t buffer[10]={0,0,0,0,0,0,0,0,0,0};
	mdc100_status_t status=mdc100_readCommand(device, MDC_100_READ_MODE, buffer);
	if(status==MDC_100_OK){
		if(buffer[2]=='+' || buffer[2]=='-'){
			 *value=(char) buffer[1];
			 return MDC_100_OK;
		}else{
			return MDC_100_CORRUPT_DATA;
		}
	}else{
		return status;
	}
}

mdc100_status_t mdc100_getKG(mdc100_t *device,int* value){
	uint8_t buffer[10]={0,0,0,0,0,0,0,0,0,0};
	mdc100_status_t status=mdc100_readCommand(device, MDC_100_READ_KG_VALUE, buffer);
	if(status==MDC_100_OK){
		uint8_t data_length  = 0;
		while(buffer[data_length]!=0){
			data_length++;
		}
		return mdc100_Str2Int(&buffer[2],data_length-1, value);
	}else{
		return status;
	}
}

mdc100_status_t mdc100_getKP(mdc100_t *device,int* value){
	uint8_t buffer[10]={0,0,0,0,0,0,0,0,0,0};
	mdc100_status_t status=mdc100_readCommand(device, MDC_100_READ_KP_VALUE, buffer);
	if(status==MDC_100_OK){
		uint8_t data_length  = 0;
		while(buffer[data_length]!=0){
			data_length++;
		}
		return mdc100_Str2Int(&buffer[2],data_length-1, value);
	}else{
		return status;
	}
}

mdc100_status_t mdc100_getKI(mdc100_t *device,int* value){
	uint8_t buffer[10]={0,0,0,0,0,0,0,0,0,0};
	mdc100_status_t status=mdc100_readCommand(device, MDC_100_READ_KI_VALUE, buffer);
	if(status==MDC_100_OK){
		uint8_t data_length  = 0;
		while(buffer[data_length]!=0){
			data_length++;
		}
		return mdc100_Str2Int(&buffer[2],data_length-1, value);
	}else{
		return status;
	}
}

mdc100_status_t mdc100_getIV(mdc100_t *device,int* value){
	uint8_t buffer[10]={0,0,0,0,0,0,0,0,0,0};
	mdc100_status_t status=mdc100_readCommand(device, MDC_100_READ_INITIAL_VALUE, buffer);
	if(status==MDC_100_OK){
		uint8_t data_length  = 0;
		while(buffer[data_length]!=0){
			data_length++;
		}
		return mdc100_Str2Int(&buffer[2],data_length-1, value);
	}else{
		return status;
	}
}


bool mdc100_readData(mdc100_t *device, uint8_t* buffer){
	osMutexAcquire(*device->mutex_uart_rx, osWaitForever);
	bool status;
	//receive the data using dma (direct memory access) to avoid excessive consumption 
	status=HAL_UART_Receive_DMA(device->huart,buffer, (TickType_t)10*portTICK_PERIOD_MS)==HAL_OK ? 1:0;
	osSemaphoreAcquire(*device->semaphore_uartIdle,  (TickType_t)10*portTICK_PERIOD_MS);
	osStatus_t semStatus=osSemaphoreAcquire(*device->semaphore_uartIdle,  (TickType_t)10*portTICK_PERIOD_MS);
	if( semStatus!=osOK || status==false){
		status=false;
	}
	HAL_UART_DMAStop(device->huart);
#if DEBUG
	printf("R:%s\n",(char *)buffer);
#endif
	osSemaphoreRelease(*device->semaphore_uartIdle);
	osMutexRelease(*device->mutex_uart_rx);
	return status;
}

bool mdc100_writeData(mdc100_t *device, uint8_t* buffer,uint8_t len){
	//lock the uart and unlock when the tx interrupt  finish
	osSemaphoreAcquire(*device->mutex_uart_tx, (TickType_t)10*portTICK_PERIOD_MS);
#if DEBUG
	printf("S:%s\n",(char *)buffer);
#endif
	//transmit the data by hardware to avoid time comsuption from the cpu
	bool status=HAL_UART_Transmit_IT(device->huart, buffer, len)==HAL_OK ? 1:0;
	return status;
}

