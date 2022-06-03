
#define _KEYA_DRIVER_H_

#pragma once


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

#define SOFT_START "!AC"
#define SET_INDIVIDUAL_DIGITS_OUT_D0 "!D0";
#define SET_INDIVIDUAL_DIGITS_OUT_D1 "!D1";
#define SOFT_STOP "!DC"
#define SAVE_CONFIG_EEPROM "!EES";
#define EMERGENCY_SHUTDOWN "!EX";
#define RELEASE_SHUTDOWN "!MG";
#define SET_MOTOR_SPEED "!M"

#define READ_MOTOR_CURRENT "?A"
#define READ_ANALOG_INPUT "?AI";
#define READ_ANALOG_INPUT_AFTER_CONVERTED "?AIC";
#define READ_BATTERY_CURRENT "?BA";
#define READ_BRUSHLESS_MOTOR_SPEED "?BS";
#define READ_INTERNAL_ANALOG "?CIA";
#define READ_INTERNAL_PULSE "?CIP";
#define READ_INTERNAL_SERIAL "?CIS";
#define READ_ALL_DIGIT_INPUTS "?D";
#define READ_SEPARATE_DIGITAL_INPUTS "?DI";
#define READ_CURRENT_DIGITAL_OUTPUTS "?DO";
#define READ_CLOSED_LOOP_ERROR "?E";
#define READ_FEEDBACK "?F";
#define READ_FAULT_MARKS "?FF"
#define READ_STATUS_FLAG "?FS"
#define READ_LOCK_STATE "?LK";
#define READ_ACTUAL_COMMAND_MOTOR "?M";
#define READ_POWER_LEVEL_APPLICATION "?P"
#define READ_PULSE_INPUT "?PI"
#define READ_PULSE_INPUT_CONVERTED "?PIC"
#define READ_ENCODER_SPEED_RPM "?S"
#define READ_TEMPERATURE "?T"
#define READ_INTERNAL_VOLTAGE "?V"



/*!
* enumeration that defines the differents status got by the send of the commands to the driver
* ok= everything is ok;
* port_close=probably the communications hasn't started;
* unknown command= the driver returns a - indicating that the command wasn't recognized
* incorrect format= the drivers replies a empty message whichs mean yu dont send a command in a format appropiate ? to read or ! to set a parameter at the start of the specific command
* need convertions= the driver returns the reading asking for and needs to be converted to its specific format
* corrupted data= the replied data can be converted because isn't in the expected way
*/
typedef enum 
{
    keya_driver_ok = 0,
    keya_driver_port_close,
    keya_driver_unknown_command,
    keya_driver_incorrect_format,
    keya_driver_need_convertion,
    keya_driver_corrupted_data,
    keya_driver_disconnected
} keya_status_t;

/**

* \brief  Struct for saving the relevant information realted to working variables of the driver.
*/
typedef struct
{
    int power[2];
    int battery_current[2];
    float temperature[3];
    float voltage[3];
    bool fault_idn[8];
    bool state_idn[8];
    bool lock_state;
} keya_info_t;



/**
 * @brief this struct handle the serial communication  variables as buffers, semaphores for interruptions 
 * for TX and RX the mutex to lock the RX and the TX and the UART handler from the STM32
 * 
 */
typedef struct{
	UART_HandleTypeDef *huart;
	osMutexId_t *uartTXmutex;
	osMutexId_t *uartRXmutex;
	osSemaphoreId_t *uartTXsem;
	osSemaphoreId_t *uartRXsem;
	int BUFFER_SIZE;
	int buffer_len;
	uint8_t buffer_reading[50];
}keya_t;

/*! 
    * \brief Cleaning the buffer of the serial device and checkout that device is really conected to the port .
    *
    * \return status_t wich informs the result of the operation
    */
keya_status_t begin(keya_t *device);

/*! 
    * \brief Checkout the reply of the driver after a sent command and determines the stae of the command sent.
    *
    * \return status_t wich informs the result of the operation*/
keya_status_t checkoutReply(char *reply);

/*! 
    * \brief Converts the data get from the driver when it need to be converted.
    *
    * \param reply A std::string with replies got from driver.
    * \param buffer A std::vector referenced to storage the data converted
    *
    * \return status_t wich informs the result of the operation
    *
    */
keya_status_t get_converted_data( char *reply,int *buffer);

/*! 
    * \brief Send a command to the driver in the appropiate format.
    *
    * \param cmd A std:string referenced, it has the command to send and storage the reply to his later convertion
    *
    * \return status_t which informs the result of the operation
    *
    */
keya_status_t send_command(keya_t *device, char *cmd);

/*! 
    * \brief Send a command to the driver in the appropiate format.
    *
    * \param cmd A std:string  it has the command to send and storage the reply to his later convertion
    * \param params A std:vector with the params of the specific command
    * \param buffer A std::vector referenced to storage the data converted
    * 
    * \return status_t wich informs the result of the operation
    *
    */
keya_status_t send_command_params_reading(keya_t *device, char* cmd, int* params, int *buffer,size_t size);

/*! 
    * \brief Send a command to the driver in the appropiate format.
    *
    * \param cmd A std:string  it has the command to send and storage the reply to his later convertion
    * \param params A std:vector with the params of the specific command
    * 
    * \return status_t wich informs the result of the operation
    *
    */
keya_status_t send_command_params(keya_t *device, char *cmd, int* params,size_t size);

/*! 
    * \brief Send a command to the driver in the appropiate format.
    *
    * \param cmd A std:string  it has the command to send and storage the reply to his later convertion
    * \param buffer A std::vector referenced to storage the data converted
    * 
    * \return status_t wich informs the result of the operation
    *
    */
keya_status_t send_command_reading(keya_t *device, char *cmd, int *buffer);

/*! 
    * \brief Get the conditions working of the driver.
    * 
    * the functions send, receive, transform and storage the relevant information of the robot
    * 
    * \return info_t with the information obtained
    *
    */
keya_info_t keya_get_info(keya_t *device);

/*! 
    * \brief Get the status of last operation that was executed.
    * 
    * \return KeyaDriverSerial::status
    *
    */
keya_status_t get_status();

/*! 
    * \brief Get the speed of the motor. 
    * 
    * the functions send, receive, transform and storage the speed of motors in rpm
    * 
    * \return std::array<float> speed of motors [0] right [1] left 
    *
    */
void get_speed(keya_t *device,int *speed);

/*! 
    * \brief Get the  power that is being applied to the motor. 
    * 
    * the functions send, receive and transform the speed of motors in rpm
    * 
    * \return std::array<int> power apllied to the motors
    *
    */
void  get_motor_power(keya_t *device, int *power);

/*! 
    * \brief Get the actual current of the motors. 
    * 
    * the functions send, receive and transform the current of motors in rpm
    * 
    * \return std::array<int> current of the motors
    *
    */
void  get_battery_current(keya_t *device , int *current);

/*! 
    * \brief Get the actual temperature.
    * 
    * the functions send, receive and transform the temperatures
    * 
    * \return std::array<float> temperatures [0]internal [1]aisle 1 surface [2] aisle 2 surface 
    *
    */
void  get_temperature(keya_t *device , float *temperature);

/*! 
    * \brief Get the actual voltages in (V) related to the driver.
    * 
    * the functions send, receive and transform  the  voltages
    * 
    * \return std::array<float> temperatures [0]internal voltage [1]main battery voltage [2] 5V voltage
    *
    */
void  get_voltage(keya_t *device , float *voltage);

/*! 
    * \brief Get the actual fault m{
arks of the driver.
    * 
    * the functions send, receive and transform fault marks of the driver
    * 
    * \return std::array<bool>  fault
    *f1 = overheat
    *f2 = Overvoltage
    *f3 = Brown
    *f4 = Short circuit
    *f5 = emergency stop
    *f6 = Sepex Excitation failure
    *f7 = MOSFET malfunction
    *f8 = Boot configuration fault
    *
    */
void get_fault_idn(keya_t *device , bool *fault);

/*! 
    * \brief Get the actual state of the driver.
    * 
    * the functions send, receive and transform state of the driver
    * 
    * \return std::array<bool>  state
    *f1 = Serial Port Profile
    *f2 = Pulse mode
    *f3 = Analog mode
    *f4 = Power stage shut-off
    *f5 = Stop Detection
    *f6 = In the limit
    *f7 = Do not use
    *f8 = Script
    *
    */
void  get_state_idn(keya_t *device, bool* state);

/*! 
    * \brief Get the lock state of the driver.
    * 
    * the functions send, receive and transform  the lock state
    * 
    * \return bool lock state of the driver
    *
    */
void get_lock_state(keya_t *device, bool * lock_state);

/*! 
    * \brief set the motor speed.
    * 
    * the functions send the speed command to the driver
    * \param int * velocity_wheels [0] Right [1] left
    * \return status_t which informs the result of the operation 
    *
    */
keya_status_t set_speed(keya_t *device,int * velocity_wheels);

keya_status_t set_soft_start(keya_t *device,int rpm_slope);
keya_status_t set_soft_stop(keya_t *device,int rpm_slope);

keya_status_t find_out(char *data_cp, int len);
void write_keya(keya_t *device,uint8_t *data, int len);
bool read_keya();



