/**************************************************************************
    This library is for the GE ChipCap2 Humidity & Temperature Sensor
    
    0 ~ 100% Relative Humidity  (7 Sec. response time)
    -40 C ~ 125 C  Temperature (5 Sec. response time)
    With Low and High alarm triggers.
    
    Various models with analog and digital output @ 3.3V or 5V operation.
    
    Digi-Key Part Number: 235-1339-ND   (Digital i2c, 5V)
    Breakout Boards available at: www.circuitsforfun.com
    Written by: Richard Wardlow @ Circuits for Fun, LLC
    GNU GPL, include above text in redistribution
***************************************************************************/

/***************************************************************************
 * This library has been modified to be compatible with stm32 microcontrollers
 * and was converted to C language 
 * Updated by: Jeison Estiven Garcia @ Robotics 4.0 S.A.S @ SunnyApp S.A.S
 * year: 2021
***************************************************************************/
#ifndef _CHIPCAP2_H_
#define _CHIPCAP2_H_

#ifdef __cplusplus
extern "C"
{
#endif
	/*!!!NO OLVIDAR DESCOMENTAR SI SE USA FREERTOS !!!*/

#define FREERTOS_ENABLED 1
#ifdef FREERTOS_ENABLED
#include "cmsis_os.h"
#endif

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f7xx_hal.h"
#define printError 0
	/**
	 * @brief structure for interaction with the pins of the stm32 device
	 * 
	 */
	typedef struct pin
	{
		GPIO_TypeDef *pin_port;
		uint16_t pin;
	} pin_t;

	/**
	 * @brief structure for configuration and stroing information for the communication and interaction with the sensor
	 * 
	 */
	typedef struct ChipCap2
	{
		uint16_t Address;
		I2C_HandleTypeDef *Hi2c_device;
		float temperature;
		float humidity;
		pin_t _pwrPin;
		pin_t _readyPin;
		pin_t _alarmLowPin;
		pin_t _alarmHighPin;
	} ChipCap2_t;

//Definitions for ChipCap2 configuration
#define END_COMMAND_MODE 0X80
#define START_COMMAND_MODE 0XA0

//Definitions for more common delays
#define DELAY_READ_EEPROM 1
#define DELAY_WRITE_EEPROM 12
#define DELAY_START_CMND 1

#define ON 1
#define OFF 0

	/**
	 * @brief enum the status of the communication protocol
	 * 
	 */
	typedef enum STATUS_BITS
	{
		VALID_DATA = 0,
		REPEAT_DATA,
		COMMAND_MODE
	} ChipCap2_status_t;

	/**
	 * @brief enum the status of the response in a process of communication 
	 * 
	 */
	typedef enum RESPONSE_BITS
	{
		COMMAND_BUSY = 0,
		COMMAND_SUCCESFUL,
		COMMAND_UNSUCCESFUL
	} ChipCap2_response_t;

	/**
	 * @brief structure for storing the information that is getted from the device to know the status of itself
	 * 
	 */
	typedef struct DIAGNOSTIC_BITS
	{
		uint8_t correctedEproomError;
		uint8_t uncorrectedEproomError;
		uint8_t ramParityError;
		uint8_t configurationError;
	} ChipCap2_diagnostic_t;

	/**
	 * @brief structure for storing the result of one command sends to the device
	 * 
	 */
	typedef struct CMND_RESULT
	{
		ChipCap2_status_t status;
		ChipCap2_response_t response;
		ChipCap2_diagnostic_t diagnostic;
		uint8_t readEproom[2];
	} ChipCap2_cmndResult_t;

	/**
	 * @brief enum the commands to read from the eeprom of the device
	 * 
	 */
	enum READ_EEPROM
	{
		READ_PDM_CLIP_H = 0x16,
		READ_PDM_CLIP_l,
		READ_ALARM_H_ON,
		READ_ALARM_H_OFF,
		READ_ALARM_L_ON,
		READ_ALARM_L_OFF,
		READ_CUST_CONFIG,
		READ_RESERVED,
		READ_CUSTOMER_ID_BYTE2,
		READ_CUSTOMER_ID_BYTE3
	};

	/**
	 * @brief enum the commands to write in the eeprom of the device
	 * 
	 */
	enum WRITE_EEPROM
	{
		WRITE_PDM_CLIP_H = 0x56,
		WRITE_PDM_CLIP_l,
		WRITE_ALARM_H_ON,
		WRITE_ALARM_H_OFF,
		WRITE_ALARM_L_ON,
		WRITE_ALARM_L_OFF,
		WRITE_CUST_CONFIG,
		WRITE_CUSTOMER_ID_BYTE2 = 0x5E,
		WRITE_CUSTOMER_ID_BYTE3
	};

	/**
	 * @brief get the temperature and humidity information coming from the device, and besides to transform the data to its respective values in celsius ans in percentage
	 * 
	 * @param device sensor to get the information from.
	 * @return ChipCap2_status_t result of the process
	 */
	ChipCap2_status_t chipCap2_readMeasurements(ChipCap2_t *device);

	/**
	 * @brief puts the device in command mode
	 * 
	 * @param device sensor to interact with.
	 * @return ChipCap2_status_t 
	 */
	ChipCap2_status_t chipCap2_startCmndMode(ChipCap2_t *device);

	/**
	 * @brief puts the device in normal mode
	 * 
	 * @param device sensor to interact with.
	 * @return ChipCap2_status_t 
	 */
	ChipCap2_status_t chipCap2_endCmndMode(ChipCap2_t *device);

	/**
	 * @brief read information about the device
	 * 
	 * @param device sensor to interact with.
	 * @param address the register which will be reading.
	 * @return ChipCap2_cmndResult_t 
	 */
	ChipCap2_cmndResult_t chipCap2_readEproom(ChipCap2_t *device, uint8_t address);

	/**
	 * @brief this functions has the protocol to change the address of the device it is important to have the device in command mode to can changes its address
	 * this process will be take effect when device is reset 
	 * @param device sensor to interact with.
	 * @param address the new address of the device
	 * @return ChipCap2_cmndResult_t 
	 */
	ChipCap2_cmndResult_t chipCap2_changeAddress(ChipCap2_t *device, uint8_t address);

	/**
	 * @brief this function makes all the process to send a specific command to the device
	 * 
	 * @param device sensor to interact with
	 * @param cmnd  the command that is going to be send 
	 * @param param a vector with the parameters of the command
	 * @return ChipCap2_cmndResult_t 
	 */
	ChipCap2_cmndResult_t chipCap2_sendCommand(ChipCap2_t *device, uint8_t cmnd, uint8_t *param);

	/**
	 * @brief transform the data getted from the sending of one command
	 * 
	 * @param resultTra structure where the information is going to be store
	 * @param result the byte that is going to be transformed.
	 */
	void chipCap2_translateResult(ChipCap2_cmndResult_t *resultTra, uint8_t result);

	/**
	 * @brief turn on or turn off the device
	 * 
	 * @param device sensor to interact with.
	 * @param state or on off
	 */
	void chipCap2_power(ChipCap2_t *device, uint8_t state);

	//functions that interact with the hardware directly this has to be change to work with another kind of microcontrollers

	/**
	 * @brief check if the device is properly connected to the microcontroller
	 * 
	 * @param device sensor to interact with
	 * @return true  if the device is connected
	 * @return false if the device is disconnected
	 */
	bool chipCap2_begin(ChipCap2_t *device);

	/**
	 * @brief send data to the device
	 * 
	 * @param device sensor to interact with
	 * @param command vector of bytes to send to the device
	 * @param len number of bytes to send
	 * @return true proccess was successful
	 * @return false proccess wasn't successful
	 */
	bool chipCap2_writeData(ChipCap2_t *device, uint8_t *command, uint8_t len);

	/**
	 * @brief read data from the device
	 * 
	 * @param device sensor to interact with
	 * @param data vector to store the data is coming from the device
	 * @param len number of bytes to read
	 * @return true proccess was successful
	 * @return false proccess wasn't successful
	 */
	bool chipCap2_readData(ChipCap2_t *device, uint8_t *data, uint8_t len);
	void chipCap2_delay(int time);

#ifdef __cplusplus
}
#endif

#endif
