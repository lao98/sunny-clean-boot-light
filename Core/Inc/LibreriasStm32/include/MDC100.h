/*
 * MDC100.h
 *
 *  Created on: Jun 17, 2021
 *      Author: Jeison Estiven Garcia
 */

#ifndef INC_MDC100_H_
#define INC_MDC100_H_

#include "stm32f7xx.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"
#include <math.h>

#define DEBUG 0

//commands
#define MDC_100_DIGITAL_MODE "D"				//
#define MDC_100_ANALOG_MODE "A"					//
#define MDC_100_CLOCKWISE_DIRECTION "+"			//MOVE THE MOTOR IN CLOCKWISE DIRECTION
#define MDC_100_COUNTER_CLOCKWISE_DIRECTION "-" //MOVE THE MOTOR IN COUNTER CLOCKWISE
#define MDC_100_START_MOTOR "S"					//
#define MDC_100_COAST ","						//SOFT BREAK OF THE MOTOR
#define MDC_100_HARD_BREAK "."					//HARD BREAK
#define MDC_100_DIGITAL_SET_POINT "M"			//CHANGES THE SET POINT SPEED OF THE DIGITAL MODE
#define MDC_100_ANALOG_SPEEDMIN_LIM "["			//CHANGES THE LOWER BOUNDARY OF ANALOG MODE SPEED
#define MDC_100_ANALOG_SPEEDMAX_LIM "]"			//CHANGES THE UPPER BOUNDARY OF THE ANALOG MODE SPEED
#define MDC_100_SET_NUMBER_POLES "P"			//INFORMATION THAT IS CRUCIAL FOR THE CORRECT RPM CALCULATIONS
#define MDC_100_ERROR_CODE "!"					//RETURNS THE ERROR CODE
#define MDC_100_FACTORY_DEFAULT "F"				//RESTORE ALL THE PARAMETERS AND VARIABLES TO DEFAULT
#define MDC_100_SET_KG_VALUE "KG"				// CHECKS AND STORES THE INITIAL GAIN CONSTANT
#define MDC_100_SET_KP_VALUE "KP"				// CHECKS AND STORES THE INITIAL PROPORTIONAL CONSTANT
#define MDC_100_SET_KI_VALUE "KI"				// CHECKS AND STORES THE INTEGRATOR CONSTANT
#define MDC_100_SET_INITIAL_VALUE "IV"			//CHECKS AND STORE THE INITIAL VALUE CONSTANT
#define MDC_100_READ_RPM_MOTOR "VM"				//RETURN THE ACTUAL SPEED THE MOTOR
#define MDC_100_READ_SET_RPM "VS"				//RETURN THE CURRENT SET POINT OF THE MOTOR
#define MDC_100_READ_SPEEDMIN_ANALOG "VN"		//RETURN THE LOWER BOUNDARY OF THE ANALOG MODE SPEED
#define MDC_100_READ_SPEEDMAX_ANALOG "VX"		//RETURN THE UPPER BOUNDARY OF THE ANALOG MODE
#define MDC_100_READ_DIGITAL_SET_RPM "VDS"		//RETURN THE SET POINT OF THE DIGITAL MODE
#define MDC_100_READ_NUMBER_POLES "VP"			//RETURN THE NUMBER OF POLES
#define MDC_100_READ_DIRECTION_MOTOR "VD"		//RETURN THE CURRENT DIRECTION OF THE MOTOR
#define MDC_100_READ_MODE "VMO"					//RETURN THE MODE OF THE CONTROLLER
#define MDC_100_READ_KG_VALUE "VKG"				//RETURN THE GAIN CONSTANT
#define MDC_100_READ_KP_VALUE "VKP"				//RETURN THE PROPORTIONAL CONSTANT
#define MDC_100_READ_KI_VALUE "VKI"				//RETURN THE INTREGATOR CONSTANT
#define MDC_100_READ_INITIAL_VALUE "VIV"		//RETURN THE INITIAL VALUE CONSTANT

/**
 * @brief enum the posibles states of the communication with the device
 * 
 */
typedef enum
{
	MDC_100_OK = 0,
	MDC_100_INVALID_COMMAND = 1,
	MDC_100_INVALID_NUMBER_CHARACTERS = 2,
	MDC_100_INSTRUCTION_NOT_AVALAIBLE = 4,
	MDC_100_WRONG_NUMBER_POLES = 8,
	MDC_100_BAD_INTEGRAL_RANGE = 16,
	MDC_100_BAD_PROPORTIONAL_RANGE = 64,
	MDC_100_BAD_RPM_RANGE = 128,
	MDC_100_BAD_INITIAL_VALUE_RANGE = 256,
	MDC_100_TIMEOUT_RX = 257,
	MDC_100_CORRUPT_DATA = 258,
	MDC_100_TRANSMIT_ERROR = 259,
	MDC_100_INVALID_OPTION = 260,
} mdc100_status_t;

/**
 * @brief structure to interact with the device and save the information getted from the communication with.
 * 
 */
typedef struct
{
	UART_HandleTypeDef *huart;
	DMA_HandleTypeDef *hdma_usart_rx;
	osSemaphoreId_t *semaphore_uartIdle;
	osMutexId_t *mutex_uart_rx;
	osSemaphoreId_t *mutex_uart_tx;
	int speed;
	int cmdSpeed;
	bool cmd;
} mdc100_t;

/**
 * @brief initialize the working of the driver
 * 
 * @param device 
 * @return mdc100_status_t 
 */
mdc100_status_t mdc100_begin(mdc100_t *device);

/**
 * @brief check if had some problem in the previous sent command 
 * 
 * @param device 
 * @return mdc100_status_t 
 */
mdc100_status_t mdc100_checkError(mdc100_t *device);

/**
 * @brief read some infprmation from the device
 * 
 * @param device 
 * @param command the command to get information required
 * @param buffer the buffer to save the bytes getted from the communication
 * @return mdc100_status_t 
 */
mdc100_status_t mdc100_readCommand(mdc100_t *device, const char *command, uint8_t *buffer);

/**
 * @brief send commands without parameters to the device
 * 
 * @param device 
 * @param command the command to send
 * @return mdc100_status_t 
 */
mdc100_status_t mdc100_writeCommand(mdc100_t *device, const char *command);

/**
 * @brief send a command with parameter to the device besides control the upper and lower limits
 * 
 * @param device 
 * @param command the command to send
 * @param param  the parameter of the command
 * @param low_limit lower limits of the parameter
 * @param upp_limit upper limits of the parameter
 * @return mdc100_status_t 
 */
mdc100_status_t mdc100_writeCommand_param(mdc100_t *device, const char *command, int param, int low_limit, int upp_limit);

/**
 * @brief utility to the library to convert string 2 to integer
 * 
 * @param buffer the string to convert 
 * @param len the length of the string
 * @param value the pointer to storage the number
 * @return mdc100_status_t 
 */
mdc100_status_t mdc100_Str2Int(uint8_t *buffer, int len, int *value);

/**
 * @brief set the mode of the driver 
 * 
 * @param device 
 * @param mode 'D' digital mode, 'A' analog mode	
 * @return mdc100_status_t 
 */
mdc100_status_t mdc100_setMode(mdc100_t *device, char mode);

/**
 * @brief set the direction of the motor
 * 
 * @param device 
 * @param direction '+' COUNTER_CLOCKWISE_DIRECTION ,'-' CLOCKWISE_DIRECTION
 * @return mdc100_status_t 
 */
mdc100_status_t mdc100_setDirection(mdc100_t *device, char direction);

/**
 * @brief Stop the motor
 * 
 * @param device 
 * @return mdc100_status_t 
 */
mdc100_status_t mdc100_softBreak(mdc100_t *device);

/**
 * @brief hard break of the motor 
 * @param device 
 * @return mdc100_status_t 
 */
mdc100_status_t mdc100_hardBreak(mdc100_t *device);

/**
 * @brief set the speed of the motor
 * 
 * @param device 
 * @param speed 
 * @return mdc100_status_t 
 */
mdc100_status_t mdc100_setSpeed(mdc100_t *device, int speed);

/**
 * @brief define the analog limits for control the speed of the motor
 * 
 * @param device 
 * @param lower_speed the lower speed limit of the motor
 * @param Upper_speed the upper speed limit of the motor
 * @return mdc100_status_t 
 */
mdc100_status_t mdc100_analogLimits(mdc100_t *device, int lower_speed, int Upper_speed);

/**
 * @brief set the number of the poles to be drived
 * 
 * @param device 
 * @param poles 
 * @return mdc100_status_t 
 */
mdc100_status_t mdc100_setPoles(mdc100_t *device, int poles);

/**
 * @brief reset from factory the driver
 * 
 * @param device 
 * @return mdc100_status_t 
 */
mdc100_status_t mdc100_factoryReset(mdc100_t *device);

/**
 * @brief set the proportional gain of the controller in the driver
 * 
 * @param device 
 * @param kg the proportional gain
 * @return mdc100_status_t 
 */
mdc100_status_t mdc100_setKG(mdc100_t *device, int kg);

/**
 * @brief set the proportional gain of the controller running in the driver 
 * 
 * @param device 
 * @param kp proportional gain value
 * @return mdc100_status_t 
 */
mdc100_status_t mdc100_setKP(mdc100_t *device, int kp);

/**
 * @brief set the intregal gain value of the controller running in the driver
 * 
 * @param device 
 * @param ki intregal gain value of the controller running in the driver
 * @return mdc100_status_t 
 */
mdc100_status_t mdc100_setKI(mdc100_t *device, int ki);

mdc100_status_t mdc100_setIV(mdc100_t *device, int iv);

/**
 * @brief get the value of the actual speed of the motor
 * 
 * @param device 
 * @param value the pointer to save the speed
 * @return mdc100_status_t 
 */
mdc100_status_t mdc100_getSpeed(mdc100_t *device, int *value);

/**
 * @brief get the command speed from the analog input
 * 
 * @param device 
 * @param value the pointer to save the data
 * @return mdc100_status_t 
 */
mdc100_status_t mdc100_getSpeedCommandAnalog(mdc100_t *device, int *value);

/**
 * @brief get the analog limits associated with the analog speed
 * 
 * @param device 
 * @param value a pointer to save the values
 * @return mdc100_status_t 
 */
mdc100_status_t mdc100_getLimitsAnalog(mdc100_t *device, int *value);

/**
 * @brief gets the actual speed digital command 
 * 
 * @param device 
 * @param value 
 * @return mdc100_status_t 
 */
mdc100_status_t mdc100_getSpeedCommandDigital(mdc100_t *device, int *value);

/**
 * @brief get the number of poles configured
 * 
 * @param device 
 * @param value a pointer to save the value
 * @return mdc100_status_t 
 */
mdc100_status_t mdc100_getPoles(mdc100_t *device, int *value);

/**
 * @brief gets the actual direction command of the motor
 * 
 * @param device 
 * @param value 
 * @return mdc100_status_t 
 */
mdc100_status_t mdc100_getDirection(mdc100_t *device, char *value);

/**
 * @brief gets the actual command mode of the driver
 * 
 * @param device 
 * @param value 
 * @return mdc100_status_t 
 */
mdc100_status_t mdc100_getMode(mdc100_t *device, char *value);

/**
 * @brief gets the actual KG of the controller
 * 
 * @param device 
 * @param value 
 * @return mdc100_status_t 
 */
mdc100_status_t mdc100_getKG(mdc100_t *device, int *value);

/**
 * @brief gets the actual KP of the controller
 * 
 * @param device 
 * @param value 
 * @return mdc100_status_t 
 */
mdc100_status_t mdc100_getKP(mdc100_t *device, int *value);

/**
 * @brief gets the actual KI of the controller
 * 
 * @param device 
 * @param value 
 * @return mdc100_status_t 
 */
mdc100_status_t mdc100_getKI(mdc100_t *device, int *value);
mdc100_status_t mdc100_getIV(mdc100_t *device, int *value);

/**
 * @brief realize the communication of low level to get data from the driver to uart communication
 * 
 * @param device 
 * @param buffer the pointer where save the data getted from the driver
 * @return true process success
 * @return false process failure
 */
bool mdc100_readData(mdc100_t *device, uint8_t *buffer);

/**
 * @brief realize the communication of low level to send data to the driver to uart communication
 * 
 * @param device 
 * @param buffer the data to send
 * @param len the numer of bytes to write
 * @return true process success
 * @return false process failure
 */
bool mdc100_writeData(mdc100_t *device, uint8_t *buffer, uint8_t len);

#endif /* INC_MDC100_H_ */
