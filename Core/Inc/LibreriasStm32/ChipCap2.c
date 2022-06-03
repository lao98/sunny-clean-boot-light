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
#include "include/ChipCap2.h"

bool chipCap2_begin(ChipCap2_t *device){
    HAL_StatusTypeDef result;
    result = HAL_I2C_IsDeviceReady(device->Hi2c_device, device->Address << 1, 2, 2);
    if (result == HAL_OK)
    { // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
    return true;
    }
    return false;
}

ChipCap2_status_t chipCap2_readMeasurements(ChipCap2_t *device)
{
	uint8_t reply[4];
	chipCap2_readData(device, &reply[0], 4);
	uint8_t status = (reply[0] & 0xC0) >> 6;
	device->humidity = (float)((((reply[0] & 0x3F) << 8) + reply[1]) / 163.84);
	device->temperature =
		(float)((reply[2] * 64) + (reply[3] >> 2)) * (165.0 / 16384.0) - 40.0;
	return status;
}

ChipCap2_status_t chipCap2_startCmndMode(ChipCap2_t *device)
{
	/**
	 * to puts the sensor in command mode is neccesary to restart it because this proccess
	 * is only possible when the device has just turn on, that is why the power pin must be connected 
	 * to a digital pin 
	 **/

	if (device->_pwrPin.pin_port != NULL)
	{
		chipCap2_power(device, OFF);
		chipCap2_delay(10);
		chipCap2_power(device, ON);
	}
	ChipCap2_cmndResult_t result = chipCap2_sendCommand(device, START_COMMAND_MODE, 0);
	return result.status;
}

ChipCap2_status_t chipCap2_endCmndMode(ChipCap2_t *device)
{
	ChipCap2_cmndResult_t result = chipCap2_sendCommand(device, END_COMMAND_MODE, 0);
	return result.status;
}

ChipCap2_cmndResult_t chipCap2_readEproom(ChipCap2_t *device, uint8_t address)
{
	ChipCap2_cmndResult_t result;
	result = chipCap2_sendCommand(device, address, 0);
	return result;
}

ChipCap2_cmndResult_t chipCap2_changeAddress(ChipCap2_t *device, uint8_t address)
{
	ChipCap2_cmndResult_t result;
	if (address < 63)
	{
		uint8_t param[2] = {0, address};
		result = chipCap2_sendCommand(device, WRITE_CUST_CONFIG, &param[0]);
		//the change take effect once you are out of command mode
	}
	return result;
}

ChipCap2_cmndResult_t chipCap2_sendCommand(ChipCap2_t *device, uint8_t cmnd, uint8_t *param)
{
	uint8_t cmndData[3];
	int delay = 0;
	ChipCap2_cmndResult_t result;
	cmndData[0] = cmnd;
	memset(&cmndData[1], 0, 2);

	if (cmnd >= WRITE_PDM_CLIP_H && cmnd <= WRITE_CUSTOMER_ID_BYTE3)
	{
		memcpy(&cmndData[1], param, 2);
		delay = DELAY_WRITE_EEPROM;
	}
	else if (cmnd == START_COMMAND_MODE)
	{
		delay = DELAY_START_CMND;
	}
	else if (cmnd >= READ_PDM_CLIP_H && cmnd <= READ_CUSTOMER_ID_BYTE3)
	{
		delay = DELAY_READ_EEPROM;
	}
	chipCap2_writeData(device, &cmndData[0], 3);
	chipCap2_delay(delay);
	uint8_t reply[3];
	if (cmnd >= READ_PDM_CLIP_H && cmnd <= READ_CUSTOMER_ID_BYTE3)
	{
		chipCap2_readData(device, &reply[0], 3);
		memcpy(&result.readEproom[0], &reply[1], 2);
	}
	else
	{
		chipCap2_readData(device, reply, 1);
	}
	chipCap2_translateResult(&result, reply[0]);
	return result;
}

void chipCap2_translateResult(ChipCap2_cmndResult_t *resultTra, uint8_t result)
{
	resultTra->status = result >> 6;
	resultTra->response = result & 0x03;
	resultTra->diagnostic.configurationError = (result >> 5) & 0x01;
	resultTra->diagnostic.ramParityError = (result >> 4) & 0x01;
	resultTra->diagnostic.uncorrectedEproomError = (result >> 3) & 0x01;
	resultTra->diagnostic.correctedEproomError = (result >> 2) & 0x01;
}

void chipCap2_power(ChipCap2_t *device, uint8_t state)
{
	if (device->_pwrPin.pin_port != NULL)
	{
		HAL_GPIO_WritePin(device->_pwrPin.pin_port, device->_pwrPin.pin, state);
	}
}

void chipCap2_delay(int time)
{
#ifdef FREERTOS_ENABLED
	osDelay(time);
#else
	HAL_Delay(time);
#endif
}

bool chipCap2_writeData(ChipCap2_t *device, uint8_t *command, uint8_t len)
{
	uint8_t status;
	status = HAL_I2C_Master_Transmit(device->Hi2c_device, device->Address << 1,
									 command, len, 10);
	if (status == HAL_OK)
	{
#if printError
		printf("Command: %x", command[0]);
		for (int i = 1; i < len; i++)
		{
			printf(" %x", command[i]);
		}
		printf("\n");
#endif
		return true;
	}
#if printError
	if (status == HAL_ERROR)
	{
		printf("HAL_I2C_Master_Transmit HAL_ERROR\r\n");
	}
	else if (status == HAL_TIMEOUT)
	{
		printf("HAL_I2C_Master_Transmit HAL_TIMEOUT\r\n");
	}
	else if (status == HAL_BUSY)
	{
		printf("HAL_I2C_Master_Transmit HAL_BUSY\r\n");
	}
	else
	{
		printf("Unknown status data %d", status);
	}

	uint32_t error = HAL_I2C_GetError(device->Hi2c_device);
	if (error == HAL_I2C_ERROR_NONE)
	{
		return;
	}
	else if (error == HAL_I2C_ERROR_BERR)
	{
		printf("HAL_I2C_ERROR_BERR\r\n");
	}
	else if (error == HAL_I2C_ERROR_ARLO)
	{
		printf("HAL_I2C_ERROR_ARLO\r\n");
	}/**
	 * @brief 
	 * 
	 * @param device 
	 * @return true 
	 * @return false 
	 */
	bool chipCap2_begin(ChipCap2_t *device);
	else if (error == HAL_I2C_ERROR_AF)
	{
		printf("HAL_I2C_ERROR_AF\r\n");
	}
	else if (error == HAL_I2C_ERROR_OVR)
	{
		printf("HAL_I2C_ERROR_OVR\r\n");
	}
	else if (error == HAL_I2C_ERROR_DMA)
	{
		printf("HAL_I2C_ERROR_DMA\r\n");
	}
	else if (error == HAL_I2C_ERROR_TIMEOUT)
	{
		printf("HAL_I2C_ERROR_TIMEOUT\r\n");
	}

	HAL_I2C_StateTypeDef state = HAL_I2C_GetState(device->Hi2c_device);
	if (state == HAL_I2C_STATE_RESET)
	{
		printf("HAL_I2C_STATE_RESET\r\n");
	}
	else if (state == HAL_I2C_STATE_READY)
	{
		printf("HAL_I2C_STATE_RESET\r\n");
	}
	else if (state == HAL_I2C_STATE_BUSY)
	{
		printf("HAL_I2C_STATE_BUSY\r\n");
	}
	else if (state == HAL_I2C_STATE_BUSY_TX)
	{
		printf("HAL_I2C_STATE_BUSY_TX\r\n");
	}
	else if (state == HAL_I2C_STATE_BUSY_RX)
	{
		printf("HAL_I2C_STATE_BUSY_RX\r\n");
	}
	else if (state == HAL_I2C_STATE_LISTEN)
	{
		printf("HAL_I2C_STATE_LISTEN\r\n");
	}
	else if (state == HAL_I2C_STATE_BUSY_TX_LISTEN)
	{
		printf("HAL_I2C_STATE_BUSY_TX_LISTEN\r\n");
	}
	else if (state == HAL_I2C_STATE_BUSY_RX_LISTEN)
	{
		printf("HAL_I2C_STATE_BUSY_RX_LISTEN\r\n");
	}
	else if (state == HAL_I2C_STATE_ABORT)
	{
		printf("HAL_I2C_STATE_ABORT\r\n");
	}
	else if (state == HAL_I2C_STATE_TIMEOUT)
	{
		printf("HAL_I2C_STATE_TIMEOUT\r\n");
	}
	else if (state == HAL_I2C_STATE_ERROR)
	{
		printf("HAL_I2C_STATE_ERROR\r\n");
	}
#endif
	return false;
}

bool chipCap2_readData(ChipCap2_t *device, uint8_t *data, uint8_t len)
{
	uint8_t status;
	status = HAL_I2C_Master_Receive(device->Hi2c_device, device->Address << 1, data, len,
									100);

	if (status == HAL_OK)
	{
#if printError
		printf("received: %x", data[0]);
		for (int i = 1; i < len; i++)
		{
			printf(" %x", data[i]);
		}
		printf("\n");
#endif
		return true;
	}
#if printError
	if (status == HAL_ERROR)
	{
		printf("HAL_I2C_Master_Transmit HAL_ERROR\r\n");
	}
	else if (status == HAL_TIMEOUT)
	{
		printf("HAL_I2C_Master_Transmit HAL_TIMEOUT\r\n");
	}
	else if (status == HAL_BUSY)
	{
		printf("HAL_I2C_Master_Transmit HAL_BUSY\r\n");
	}
	else
	{
		printf("Unknown status data %d", status);
	}

	uint32_t error = HAL_I2C_GetError(device->Hi2c_device);
	if (error == HAL_I2C_ERROR_NONE)
	{
		return;
	}
	else if (error == HAL_I2C_ERROR_BERR)
	{
		printf("HAL_I2C_ERROR_BERR\r\n");
	}
	else if (error == HAL_I2C_ERROR_ARLO)
	{
		printf("HAL_I2C_ERROR_ARLO\r\n");
	}
	else if (error == HAL_I2C_ERROR_AF)
	{
		printf("HAL_I2C_ERROR_AF\r\n");
	}
	else if (error == HAL_I2C_ERROR_OVR)
	{
		printf("HAL_I2C_ERROR_OVR\r\n");
	}
	else if (error == HAL_I2C_ERROR_DMA)
	{
		printf("HAL_I2C_ERROR_DMA\r\n");
	}
	else if (error == HAL_I2C_ERROR_TIMEOUT)
	{
		printf("HAL_I2C_ERROR_TIMEOUT\r\n");
	}

	HAL_I2C_StateTypeDef state = HAL_I2C_GetState(device->Hi2c_device);
	if (state == HAL_I2C_STATE_RESET)
	{
		printf("HAL_I2C_STATE_RESET\r\n");
	}
	else if (state == HAL_I2C_STATE_READY)
	{
		printf("HAL_I2C_STATE_RESET\r\n");
	}
	else if (state == HAL_I2C_STATE_BUSY)
	{
		printf("HAL_I2C_STATE_BUSY\r\n");
	}
	else if (state == HAL_I2C_STATE_BUSY_TX)
	{
		printf("HAL_I2C_STATE_BUSY_TX\r\n");
	}
	else if (state == HAL_I2C_STATE_BUSY_RX)
	{
		printf("HAL_I2C_STATE_BUSY_RX\r\n");
	}
	else if (state == HAL_I2C_STATE_LISTEN)
	{
		printf("HAL_I2C_STATE_LISTEN\r\n");
	}
	else if (state == HAL_I2C_STATE_BUSY_TX_LISTEN)
	{
		printf("HAL_I2C_STATE_BUSY_TX_LISTEN\r\n");
	}
	else if (state == HAL_I2C_STATE_BUSY_RX_LISTEN)
	{
		printf("HAL_I2C_STATE_BUSY_RX_LISTEN\r\n");
	}
	else if (state == HAL_I2C_STATE_ABORT)
	{
		printf("HAL_I2C_STATE_ABORT\r\n");
	}
	else if (state == HAL_I2C_STATE_TIMEOUT)
	{
		printf("HAL_I2C_STATE_TIMEOUT\r\n");
	}
	else if (state == HAL_I2C_STATE_ERROR)
	{
		printf("HAL_I2C_STATE_ERROR\r\n");
	}
#endif
	return false;
}
