/*
 * Developer: Bud Ryerson https://github.com/budryerson/TFMini-Plus-I2C/
 * Date:      03 SEP 2020
 * Version:   1.5.0
 * Described: Arduino Library for the Benewake TFMini-Plus Lidar sensor
 *            configured for the I2C interface
 * 
 * Developer Update: Jeison Estiven Garcia @ Robotics 4.0 S.A.S @SunnyApp S.A.S
 * Date:      2021
 * Described: STM32 C Library for the Benewake TF02-PRO Lidar sensor
 *            configured for the I2C interface
 */
#ifndef TF02PRO_H_
#define TF02PRO_H_

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

#define TF_DEFAULT_ADDRESS 0x10
// Buffer size definitions
#define TF_FRAME_SIZE 9  // Size of data frame = 9 bytes
#define TF_REPLY_SIZE 8  // Longest command reply = 8 bytes
#define TF_COMMAND_MAX 8 // Longest command = 8 bytes

  // Command Definitions
  /* - - - - -  TF02 PRO & Command Formats  - - - - -
  Data Frame format:
  Byte0  Byte1  Byte2   Byte3   Byte4   Byte5   Byte6   Byte7   Byte8
  0x59   0x59   Dist_L  Dist_H  Flux_L  Flux_H  Temp_L  Temp_H  CheckSum_
  Data Frame Header character: Hex 0x59, Decimal 89, or "Y"
  Command format:
  Byte0  Byte1   Byte2   Byte3 to Len-2  Byte Len-1
  0x5A   Length  Cmd ID  Payload if any   Checksum
 - - - - - - - - - - - - - - - - - - - - - - - - - */

  // The library 'sendCommand( cmnd, param)' function
  // defines a command (cmnd) in the the following format:
  // 0x     00       00       00       00
  //     one byte  command  command   reply
  //     payload   number   length    length

#define ENTER_CONFIG_MODE 0x0200000100F055AA
#define EXIT_CONFIG_MODE 0x0200000000F055AA
#define SET_SERIAL_MODE 0x000A0500 // return no reply data
#define SET_I2C_MODE 0x010A0500    //           "

#define GET_FIRMWARE_VERSION 0xA000000000F055AA // return 3 byte firmware version

#define SET_FRAME_RATE 0x00030606         // return an echo of the command
#define STANDARD_FORMAT_CM 0x01050505     //           "
#define STANDARD_FORMAT_MM 0x06050505     //           "
#define STANDARD_FORMAT_STRING 0x02050505 //           "
#define SET_BAUD_RATE 0x00060808          //           "
#define ENABLE_OUTPUT 0x00070505          //           "
#define DISABLE_OUTPUT 0x01070505         //           "
#define SET_I2C_ADDRESS 0x100B0505        //           "

#define SOFT_RESET 0xFFFFFFFF00F055AA // echo and pass(0)/fail(1) byte
#define FACTORY_RESET 0x00100405      //           "

#define I2C_FORMAT_CM 0x01000500     // return 9 byte data frame
#define I2C_FORMAT_MM 0x06000500     //           "
#define TRIGGER_DETECTION 0x00040400 // return 9 byte serial data 
                                     // frame rate set to zero

// Command Parameter Definitions
// (generally not used in I2C Communications Mode)
#define BAUD_9600 0x002580  // UART serial baud rate
#define BAUD_14400 0x003840 // expressed in hexidecimal
#define BAUD_19200 0x004B00
#define BAUD_56000 0x00DAC0
#define BAUD_115200 0x01C200
#define BAUD_460800 0x070800
#define BAUD_921600 0x0E1000

#define FRAME_0 0x0000 // internal measurement rate
#define FRAME_1 0x0001 // expressed in hexidecimal
#define FRAME_2 0x0002
#define FRAME_5 0x0003
#define FRAME_10 0x000A
#define FRAME_20 0x0014
#define FRAME_25 0x0019
#define FRAME_50 0x0032
#define FRAME_100 0x0064
#define FRAME_125 0x007D
#define FRAME_200 0x00C8
#define FRAME_250 0x00FA
#define FRAME_500 0x01F4
#define FRAME_1000 0x03E8

  typedef struct tf02PRO
  {
    uint16_t Address;
    uint16_t Distance;
    uint16_t Strength;
    uint16_t Temp;
    I2C_HandleTypeDef *Hi2c_device;
  } tf02PRO_t;

  /**
   * @brief send data to the device through the I2C interface
   * 
   * @param device sensor to interact with.
   * @param command vector with the bytes will be sent.
   * @param len number of bytes will be transmitted.
   * @return true process success.
   * @return false process failure.
   */
  bool tf02Pro_writeData(tf02PRO_t *device, uint8_t *command, uint8_t len);

  /**
   * @brief read data from the device through the I2C interface
   * 
   * @param device sensor to interact with.
   * @param data pointer where the bytes coming will be saved.
   * @param len  number of bytes will be read.
   * @return true 
   * @return false 
   */
  bool tf02Pro_readData(tf02PRO_t *device, uint8_t *data, uint8_t len);

  /**
   * @brief delay of the microcontroller for this library 
   * 
   * @param time 
   */
  void tf02Pro_delay(int time);

  /**
   * @brief check the device connection.
   * 
   * @param device 
   * @return true device is connected.
   * @return false device is not connected.
   */
  bool tf02Pro_begin(tf02PRO_t *device);

  /**
   * @brief get the distance data in mm
   * 
   * @param device 
   * @return true the process success
   * @return false the process failure
   */
  bool tf02Pro_getDataMM(tf02PRO_t *device);

  /**
   * @brief get the distance data in cm
   * 
   * @param device 
   * @return true the process success
   * @return false the process failure
   */
  bool tf02Pro_getDataCM(tf02PRO_t *device);

  /**
   * @brief making the send of command to get the distance data.
   * 
   * @param device 
   * @param cmnd the command to get the distance data in mm or cm 
   * @return true 
   * @return false 
   */
  bool tf02Pro_getData(tf02PRO_t *device, uint32_t cmnd);

  /**
   * @brief realize the process to send the command with parameters to the device 
   * 
   * @param cmnd the command will be sent 
   * @param param the parameters of the command like address or baud rate
   * @param device 
   * @return true 
   * @return false 
   */
  bool tf02Pro_sendCommand32(uint32_t cmnd, uint32_t param, tf02PRO_t *device);

  /**
   * @brief realize the process to send large commands without parameters like puts in configuration mode or get the firmware version
   * 
   * @param cmnd the command will be sent 
   * @param device 
   * @return true 
   * @return false 
   */
  bool tf02Pro_sendCommand64(uint64_t cmnd, tf02PRO_t *device);

#ifdef __cplusplus
}
#endif

#endif // tf02Pro_H_
