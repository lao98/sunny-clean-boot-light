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
#ifndef _SPARKFUN_QWIIC_RELAY_H_
#define _SPARKFUN_QWIIC_RELAY_H_

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
#define PRINTERROR 0
  typedef struct Qwiic_Relay
  {
    uint16_t Address;
    I2C_HandleTypeDef *Hi2c_device;
  } Qwiic_Relay_t;

  enum SF_AVAILABLE_QUAD_RELAYS
  {

    RELAY_ONE = 0x01,
    RELAY_TWO,
    RELAY_THREE,
    RELAY_FOUR,

  };

  enum SF_QUAD_RELAY_COMMANDS
  {

    TOGGLE_RELAY_ONE = 0x01,
    TOGGLE_RELAY_TWO,
    TOGGLE_RELAY_THREE,
    TOGGLE_RELAY_FOUR,
    RELAY_ONE_STATUS,
    RELAY_TWO_STATUS,
    RELAY_THREE_STATUS,
    RELAY_FOUR_STATUS,
    TURN_ALL_OFF = 0xA,
    TURN_ALL_ON,
    TOGGLE_ALL,
    RELAY_ONE_PWM = 0x10,
    RELAY_TWO_PWM,
    RELAY_THREE_PWM,
    RELAY_FOUR_PWM
  };

  enum SF_QUAD_RELAY_STATUS
  {

    QUAD_RELAY_OFF = 0,
    QUAD_RELAY_ON = 15

  };

  enum SF_SINGLE_RELAY_COMMANDS
  {

    TURN_RELAY_OFF = 0x00,
    TURN_RELAY_ON,
    FIRMWARE_VERSION = 0x04,
    MYSTATUS

  };

  enum SF_SINGLE_RELAY_STATUS
  {

    SING_RELAY_OFF = 0,
    SING_RELAY_ON = 1

  };

#define QUAD_DEFAULT_ADDRESS 0x6D
#define QUAD_ALTERNATE_ADDRESS 0x6C
#define SINGLE_DEFAULT_ADDRESS 0x18
#define SINGLE_ALTERNATE_ADDRESS 0x19

#define QUAD_SSR_DEFAULT_ADDRESS 0x08
#define QUAD_SSR_ALTERNATE_ADDRESS 0x09
#define DUAL_SSR_DEFAULT_ADDRESS 0x0A
#define DUAL_SSR_ALTERNATE_ADDRESS 0x0B

#define ADDRESS_LOCATION 0X03 //0xC7
#define INCORR_PARAM 0xFF

  bool Qwiic_Relay_begin(Qwiic_Relay_t *device); // begin function

  //****----THE FOLLOWING FIVE FUNCTIONS ARE TO BE USED WITH THE SPARKFUN SINGLE RELAY-----****

  // This function gets the version number of the SparkFun Single Relay.
  float Qwiic_Relay_singleRelayVersion(Qwiic_Relay_t *device);

  //This function starts a slow PWM (1Hz) with a range from 0-100 as this is the maximum PWM resolution for Zero-crossing SSR's at 50Hz
  bool Qwiic_Relay_setSlowPWM(Qwiic_Relay_t *device, uint8_t relay, uint8_t pwmValue);

  //This function starts a slow PWM (1Hz) with a range from 0-100 as this is the maximum PWM resolution for Zero-crossing SSR's at 50Hz
  uint8_t Qwiic_Relay_getSlowPWM(Qwiic_Relay_t *device, uint8_t relay);

  //*****----THE FOLLOWING FUNCTIONS ARE TO BE USED WITH THE SPARKFUN QUAD RELAY------*****

  // This function turns the given relay on. While this also works for the
  // SparkFun Single Relay, it is meant for the SparkFun Quad Relay.
  void Qwiic_Relay_turnRelayOn(Qwiic_Relay_t *device, uint8_t relay);

  // This function turns the given relay off.
  void Qwiic_Relay_turnRelayOff(Qwiic_Relay_t *device, uint8_t relay);

  // This function toggles the given relay. If the relay is on then it will turn
  // it off, and if it is off then it will turn it on.
  void Qwiic_Relay_toggleRelay(Qwiic_Relay_t *device, uint8_t relay);

  // This function for the SparkFun Quad Relay, turns on all relays on the
  // board.
  void Qwiic_Relay_turnAllRelaysOn(Qwiic_Relay_t *device);

  // This function for the SparkFun Quad Relay, turns off all relays on the
  // board.
  void Qwiic_Relay_turnAllRelaysOff(Qwiic_Relay_t *device);

  // This function for the SparkFun Quad Relay, turns off all relays on the
  // board.
  void Qwiic_Relay_toggleAllRelays(Qwiic_Relay_t *device);

  // This function for the SparkFun Quad Relay, gets the status of the relay:
  // whether on: 1 or off: 0;
  uint8_t Qwiic_Relay_getState(Qwiic_Relay_t *device, uint8_t relay);

  // This function changes the I-squared-C address of the Qwiic RFID. The address
  // is written to the memory location in EEPROM that determines its address.
  bool Qwiic_Relay_changeAddress(Qwiic_Relay_t *device, uint8_t newAddress);

  //This function writes a value to an address in the relay, used to set PWM values
  bool Qwiic_Relay_writeAddress(Qwiic_Relay_t *device, uint8_t address, uint8_t value);

  // This function handles I-squared-C write commands for turning the relays on.
  // The quad relay relies on the current state of the relay to determine whether
  // or not to turn the respective relay on (or off) and so the current state of
  // the relay is checked before attempting to send a command.
  void Qwiic_Relay_writeCommandOn(Qwiic_Relay_t *device, uint8_t _command);

  // This function handles I-squared-C write commands for turning the relays off.
  // The quad relay relies on the current state of the relay to determine whether
  // or not to turn the respective relay off (or on) and so the current state of
  // the relay is checked before attempting to toggle it.
  void Qwiic_Relay_writeCommandOff(Qwiic_Relay_t *device, uint8_t _command);

  // This command sends the I-squared-C write command to toggle relays from their
  // current state.
  void Qwiic_Relay_writeCommandToggle(Qwiic_Relay_t *device, uint8_t _command);

  // This function requests information from the product with a simple
  // I-squared-C transaction.
  uint8_t Qwiic_Relay_readCommand(Qwiic_Relay_t *device, uint8_t _command);

  // The function reads thee version number of the Single Quad Relay.
  float Qwiic_Relay_readVersion(Qwiic_Relay_t *device, uint8_t _command);

  bool Qwiic_Relay_writeData(Qwiic_Relay_t *device, uint8_t *command, uint8_t len);
  bool Qwiic_Relay_readData(Qwiic_Relay_t *device, uint8_t *data, uint8_t len);
  void Qwiic_Relay_delay(int time);

#ifdef __cplusplus
}
#endif

#endif
