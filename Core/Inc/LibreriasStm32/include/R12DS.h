/*
 * r9ds.h
 *
 *  Created on: 2022
 *      Author: pc
 */

#ifndef INC_R12DS_H_
#define INC_R12DS_H_


#include "stm32f7xx.h"
#include "cmsis_os.h"

#include <stdbool.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include <stdlib.h>

#define R12DS_STARTBYTE 0x0F
#define R12DS_FOOTERBYTE 0
#define R12DS_DEBUG 1


#define  MAX_SPEED_LINEAR  1.0
#define  MIN_SPEED_LINEAR -1.0
#define  MAX_SPEED_ANGULAR  1.57
#define  MIN_SPEED_ANGULAR  -1.57
#define  MAX_SPEED_BRUSH  204
#define  MIN_SPEED_BRUSH  -204
#define  UPPER_LIMIT_CHANNELS 1694// superior 1694
#define  LOWER_LIMIT_CHANNELS 310// inferior 306
#define  ZERO_CHANNELS 1000
#define  THRESHOLD  100 // limite

#define CH_ANG_VEL  3
#define CH_LIN_VEL  1
#define CH_L_R  0
#define CH_U_D  2
#define CH_ELECVAL 4
#define CH_ONOFF  5
#define CH_ELECVAL_ONOFF  6
#define CH_BRU0_VEL  7
#define CH_BRU1_VEL  8
#define CH_BRU_ONOFF  9
#define CH_BRAKE  10
#define CH_BRU 11

typedef struct 
{
	bool change;
	int value;
}values_int_t;

typedef struct 
{
	bool change;
	float value;
}values_float_t;
/**
 * \brief struct to save the data received and translated for its later use.
 *  \param onOFF= Turn on or off the robot activity
 *  \param brake= Stop the move of the robot
 *  \param elecValOnOff= Enable or disable the electrovalves
 *  \param brushesOnOff= Enable or disable the brushes
 *  \param elecVal = which electrovalves will be on 0.all off 1.front 2.both 3.back
 *  \param brush = which brushes will be on 0.all off 0.1.front 2.both 3.back
 *  \param frontSBrushpeed= speed command for the front brush
 *  \param backBrushSpeed= speed command for the front brush
 *  \param robotLinearVelocity= linear velocity command for the robot
 *  \param robotAngularVelocity= angular velocity command for the robot
 */
typedef struct
{
	values_int_t onOff;
	values_int_t brake;
	values_int_t elecValOnOff;
	values_int_t brushesOnOff;
	values_int_t elecVal;
	values_int_t brush;

	values_int_t frontBrushSpeed;
	values_int_t backBrushSpeed;
	values_float_t robotLinearVelocity;
	values_float_t robotAngularVelocity;
}data_t;

typedef struct
{
	float max_speed_linear;
	float min_speed_linear;
	float max_speed_angular;
	float min_speed_angular;
	float max_speed_brush;
	float min_speed_brush;
	int upper_limit_channels;
	int lower_limit_channels;
	int zero_channels;
	int threshold;
}rf_t;
/**
 * @brief enum the different status that the rf receiver could be found
 * 
 */
typedef enum{
	R12DS_OK=0,
	R12DS_BAD_HEADER,
	R12DS_BAD_FOOTER,
	R12DS_FRAME_LOST,
	R12DS_FAILSAFE_ACTIVATED,
	R12DS_RX_TIMEOUT
}r9ds_status_t;

/**
 * @brief structure for interacting with the device and save the status and the channels data
 * 
 */
typedef struct{
	//uart interface
	UART_HandleTypeDef *huart_r9ds;
	//direct memory access
	//DMA_HandleTypeDef *hdma_usart_rx;
	//semaphore to controk the idle line for uart event.
	osSemaphoreId_t *semaphore_uartIdle;
	//mutex to block the communication process
	osMutexId_t *mutex_uart_rx;
	uint8_t buffer[25];
	uint8_t buffer_rf[26];
	//int BUFFER_SIZE;
	//int buffer_len;
	int channels[10];
	float control;
	float control_w;

	r9ds_status_t status;
}r9ds_t;


/**
 * @brief Function fot getting and transforming the data of the channels of the sbus communication with the device
 * 
 * @param device 
 * @return r12ds_status_t 
 */
r9ds_status_t r9ds_getChannels(r9ds_t *device);

/**
 * @brief Realize the communication through the uart bus
 * 
 * @param device 
 * @param buffer pointer where the data coming from the communication will be saved
 * @return true  process success
 * @return false process failure
 */
bool r9ds_readData(r9ds_t *device);//,uint8_t * buffer

/** \brief Look for changes in the parameters and update the variables
*/
void updateCallback();

/** \brief convert the data from switches
 */
int convertSwitch(int value);

/** \brief convert the data from three state switches
 */
int convert3Switch(int value);

/** \brief convert the data from know in his respective value
 */
float convertKnob(int value, float lower_lim, float upper_lim);

#endif /* INC_r12ds_H_ */

