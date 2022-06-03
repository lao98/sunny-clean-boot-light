/***************************************************************************
 The MIT License (MIT)

Copyright (c) 2016 SparkFun Electronics
Updated in 2021 by Jeison Garcia working for Robotics 4.0 S.A.S 
which is outsourced by SunnyApp S.A.S

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal 
in the Software without restriction, including without limitation the rights 
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell 
copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION 
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *************************************************************************/
/*
  This is a library for both the Qwiic Single and Qwiic Quad Relay. It gives basic
  functionality for turning on and off your relays and getting their statuses. 
  By: Elias Santistevan
  Date: July 2019 
  
  this has been modified to be version STM32 and to the C programming language
  By: Jeison Estiven Garcia
  Date: March 2021
 */

#include "include/SparkFun_Qwiic_Relay.h"

bool Qwiic_Relay_begin(Qwiic_Relay_t *device)
{
  HAL_StatusTypeDef result;
  result = HAL_I2C_IsDeviceReady(device->Hi2c_device, device->Address << 1, 2, 2);
  if (result == HAL_OK)
  { // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
    return true;
  }
  return false;
}

//****----THE FOLLOWING FIVE FUNCTIONS ARE TO BE USED WITH THE SPARKFUN SINGLE RELAY-----****

// This function gets the version number of the SparkFun Single Relay.
float Qwiic_Relay_singleRelayVersion(Qwiic_Relay_t *device)
{
  float version = Qwiic_Relay_readVersion(device, FIRMWARE_VERSION);
  return (version);
}

//This function starts a slow PWM (1Hz) with a range from 0-100 so as this is the maximum PWM resolution for Zero-crossing SSR's at 50Hz
bool Qwiic_Relay_setSlowPWM(Qwiic_Relay_t *device, uint8_t relay, uint8_t pwmValue)
{
  return Qwiic_Relay_writeAddress(device, RELAY_ONE_PWM + relay - 1, pwmValue);
}

//This function starts a slow PWM (1Hz) with a range from 0-100 so as this is the maximum PWM resolution for Zero-crossing SSR's at 50Hz
uint8_t Qwiic_Relay_getSlowPWM(Qwiic_Relay_t *device, uint8_t relay)
{
  return Qwiic_Relay_readCommand(device, RELAY_ONE_PWM + relay - 1);
}

//*****----THE FOLLOWING FUNCTIONS ARE TO BE USED WITH THE SPARKFUN QUAD RELAY------*****

// This function turns the given relay on.
void Qwiic_Relay_turnRelayOn(Qwiic_Relay_t *device, uint8_t relay)
{
  if (!relay)
  {
    Qwiic_Relay_writeCommandOn(device, TURN_RELAY_ON);
  }
  else
  {
    Qwiic_Relay_writeCommandOn(device, relay);
  }
}

// This function turns the given relay off.
void Qwiic_Relay_turnRelayOff(Qwiic_Relay_t *device, uint8_t relay)
{
  if (!relay)
  {
    Qwiic_Relay_writeCommandOff(device, TURN_RELAY_OFF);
  }
  else
  {
    Qwiic_Relay_writeCommandOff(device, relay);
  }
}

// This function toggles the given relay. If the relay is on then it will turn
// it off, and if it is off then it will turn it on.
void Qwiic_Relay_toggleRelay(Qwiic_Relay_t *device, uint8_t relay)
{
  if (!relay)
  {
    uint8_t status = Qwiic_Relay_readCommand(device, MYSTATUS);
    if (status == SING_RELAY_ON)
      Qwiic_Relay_turnRelayOff(device, 0);
    else
      Qwiic_Relay_turnRelayOn(device, 0);
  }
  else
  {
    if (relay == RELAY_ONE)
      Qwiic_Relay_writeCommandToggle(device, TOGGLE_RELAY_ONE);
    else if (relay == RELAY_TWO)
      Qwiic_Relay_writeCommandToggle(device, TOGGLE_RELAY_TWO);
    else if (relay == RELAY_THREE)
      Qwiic_Relay_writeCommandToggle(device, TOGGLE_RELAY_THREE);
    else if (relay == RELAY_FOUR)
      Qwiic_Relay_writeCommandToggle(device, TOGGLE_RELAY_FOUR);
    else
      return;
  }
}

// This function for the SparkFun Quad Relay, turns on all relays on the
// board.
void Qwiic_Relay_turnAllRelaysOn(Qwiic_Relay_t *device)
{
  Qwiic_Relay_writeCommandOn(device, TURN_ALL_ON);
}

// This function for the SparkFun Quad Relay, turns off all relays on the
// board.
void Qwiic_Relay_turnAllRelaysOff(Qwiic_Relay_t *device)
{
  Qwiic_Relay_writeCommandOff(device, TURN_ALL_OFF);
}

// This function for the SparkFun Quad Relay, turns off all relays on the
// board.
void Qwiic_Relay_toggleAllRelays(Qwiic_Relay_t *device)
{
  Qwiic_Relay_writeCommandOn(device, TOGGLE_ALL);
}

// This function for the SparkFun Quad Relay, gets the status of the relay:
// whether on: 1 or off: 0;
uint8_t Qwiic_Relay_getState(Qwiic_Relay_t *device, uint8_t relay)
{

  uint8_t status;
  if (!relay)
  {
    status = Qwiic_Relay_readCommand(device, MYSTATUS);
    return status;
  }
  if (relay == RELAY_ONE)
    status = Qwiic_Relay_readCommand(device, RELAY_ONE_STATUS);
  else if (relay == RELAY_TWO)
    status = Qwiic_Relay_readCommand(device, RELAY_TWO_STATUS);
  else if (relay == RELAY_THREE)
    status = Qwiic_Relay_readCommand(device, RELAY_THREE_STATUS);
  else if (relay == RELAY_FOUR)
    status = Qwiic_Relay_readCommand(device, RELAY_FOUR_STATUS);
  else
    return INCORR_PARAM;

  if (status == QUAD_RELAY_ON) // Relay status should be consistent
    return 1;                  // Relay on
  else
    return QUAD_RELAY_OFF;
}

// This function changes the I-squared-C address of the Qwiic RFID. The address
// is written to the memory location in EEPROM that determines its address.
bool Qwiic_Relay_changeAddress(Qwiic_Relay_t *device, uint8_t newAddress)
{

  if (newAddress < 0x07 || newAddress > 0x78) // Range of legal addresses
    return false;

  uint8_t command[2] = {ADDRESS_LOCATION, newAddress};
  return Qwiic_Relay_writeData(device, &command[0], 2);
}

// This function writes a value to an address
bool Qwiic_Relay_writeAddress(Qwiic_Relay_t *device, uint8_t addressToWrite, uint8_t value)
{
  uint8_t command[2] = {addressToWrite, value};
  return Qwiic_Relay_writeData(device, &command[0], 2);
}
// This function handles I-squared-C write commands for turning the relays on.
// The quad relay relies on the current state of the relay to determine whether
// or not to turn the respective relay on (or off) and so the current state of
// the relay is checked before attempting to send a command.
void Qwiic_Relay_writeCommandOn(Qwiic_Relay_t *device, uint8_t _command)
{
  uint8_t _toggleRelay;
  uint8_t _status;
  if (_command == RELAY_ONE)
  {
    _status = Qwiic_Relay_readCommand(device, RELAY_ONE_STATUS);
    if (_status == QUAD_RELAY_ON)
    {         // Is the board off?
      return; // Do nothing...
    }
    else
    { // Then it must be on...
      _toggleRelay = TOGGLE_RELAY_ONE;
    }
  }
  // Repeat for relay two....
  else if (_command == RELAY_TWO)
  {
    _status = Qwiic_Relay_readCommand(device, RELAY_ONE_STATUS);
    if (_status == QUAD_RELAY_ON)
    {
      return;
    }
    else
    {
      _toggleRelay = TOGGLE_RELAY_TWO;
    }
  }
  // Relay three...
  else if (_command == RELAY_THREE)
  {
    _status = Qwiic_Relay_readCommand(device, RELAY_THREE_STATUS);
    if (_status == QUAD_RELAY_ON)
    {
      return;
    }
    else
    {
      _toggleRelay = TOGGLE_RELAY_THREE;
    }
  }
  // Relay four....
  else if (_command == RELAY_FOUR)
  {
    _status = Qwiic_Relay_readCommand(device, RELAY_FOUR_STATUS);
    if (_status == QUAD_RELAY_ON)
    {
      return;
    }
    else
    {
      _toggleRelay = TOGGLE_RELAY_FOUR;
    }
  }
  // If it's not 1-4 then it must be for the single relay...
  else
  {
    Qwiic_Relay_writeData(device, &_command, 1);
    return;
  }
  Qwiic_Relay_writeData(device, &_toggleRelay, 1);
}

// This function handles I-squared-C write commands for toggling the relays from their
// current state. If the relay is on then it will be turned off and vice versa.
void Qwiic_Relay_writeCommandToggle(Qwiic_Relay_t *device, uint8_t _command)
{
  Qwiic_Relay_writeData(device, &_command, 1);
}

// This function handles I-squared-C write commands for turning the relays off.
// The quad relay relies on the current state of the relay to determine whether
// or not to turn the respective relay off (or on) and so the current state of
// the relay is checked before attempting to toggle it.
void Qwiic_Relay_writeCommandOff(Qwiic_Relay_t *device, uint8_t _command)
{
  uint8_t _toggleRelay;
  uint8_t _status;
  if (_command == RELAY_ONE)
  {
    _status = Qwiic_Relay_readCommand(device, RELAY_ONE_STATUS);
    if (_status == QUAD_RELAY_OFF)
    {         // Is the board off?
      return; // Do nothing...
    }
    else
    { // Then it must be on...
      _toggleRelay = TOGGLE_RELAY_ONE;
    }
  }
  // Repeat for relay two....
  else if (_command == RELAY_TWO)
  {
    _status = Qwiic_Relay_readCommand(device, RELAY_ONE_STATUS);
    if (_status == QUAD_RELAY_OFF)
    {
      return;
    }
    else
    {
      _toggleRelay = TOGGLE_RELAY_TWO;
    }
  }
  // Relay three...
  else if (_command == RELAY_THREE)
  {
    _status = Qwiic_Relay_readCommand(device, RELAY_THREE_STATUS);
    if (_status == QUAD_RELAY_OFF)
    {
      return;
    }
    else
    {
      _toggleRelay = TOGGLE_RELAY_THREE;
    }
  }
  // Relay four....
  else if (_command == RELAY_FOUR)
  {
    _status = Qwiic_Relay_readCommand(device, RELAY_FOUR_STATUS);
    if (_status == QUAD_RELAY_OFF)
    {
      return;
    }
    else
    {
      _toggleRelay = TOGGLE_RELAY_FOUR;
    }
  }
  // If it's not 1-4 then it must be for the single relay...
  else
  {
    Qwiic_Relay_writeData(device, &_command, 1);
    return;
  }
  Qwiic_Relay_writeData(device, &_toggleRelay, 1);
}

// This function requests information from the product with a simple
// I-squared-C transaction.
uint8_t Qwiic_Relay_readCommand(Qwiic_Relay_t *device, uint8_t _command)
{

  Qwiic_Relay_writeData(device, &_command, 1);
  uint8_t reply;
  Qwiic_Relay_readData(device, &reply, 1);
  uint8_t status = reply;
  return (status);
}

// The function reads thee version number of the Single Quad Relay.
float Qwiic_Relay_readVersion(Qwiic_Relay_t *device, uint8_t _command)
{
  Qwiic_Relay_writeData(device, &_command, 1);
  uint8_t reply[2];
  Qwiic_Relay_readData(device, &reply[0], 2);
  float _versValue = reply[0];
  _versValue += (float)reply[1] / 10.0;
  return (_versValue);
}

void Qwiic_Relay_delay(int time)
{
#ifdef FREERTOS_ENABLED
  osDelay(time);
#else
  HAL_Delay(time);
#endif
}

bool Qwiic_Relay_writeData(Qwiic_Relay_t *device, uint8_t *command, uint8_t len)
{
  uint8_t status;
  status = HAL_I2C_Master_Transmit(device->Hi2c_device, device->Address << 1,
                                   command, len, 10);
  if (status == HAL_OK)
  {
#if PRINTERROR
    printf("Command: %x", command[0]);
    for (int i = 1; i < len; i++)
    {
      printf(" %x", command[i]);
    }
    printf("\n");
#endif
    return true;
  }

#if PRINTERROR
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

bool Qwiic_Relay_readData(Qwiic_Relay_t *device, uint8_t *data, uint8_t len)
{
  uint8_t status;
  status = HAL_I2C_Master_Receive(device->Hi2c_device, device->Address << 1, data, len,
                                  100);

  if (status == HAL_OK)
  {
#if PRINTERROR
    printf("received: %x", data[0]);
    for (int i = 1; i < len; i++)
    {
      printf(" %x", data[i]);
    }
    printf("\n");
#endif
    return true;
  }
#if PRINTERROR
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
