/*
MIT License

Copyright (c) 2019 Ivy Knob https://github.com/ivyknob/bno055_stm32/

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

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#ifndef BNO055_H_
#define BNO055_H_

#ifdef __cplusplus
extern "C"
{
#endif
  // #define FREERTOS_ENABLED true

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "stm32f7xx_hal.h"

//Compiler definition for printError in the working of the communication protocol, helps to debug the communications when is active
#define printError 0

//Compiler defintion to make the code compatible to work with FREERTOS
#define FREERTOS_ENABLED 1

#ifdef FREERTOS_ENABLED
#include "cmsis_os.h"
#endif

#define START_BYTE 0xAA
#define RESPONSE_BYTE 0xBB
#define ERROR_BYTE 0xEE

#define BNO055_I2C_ADDR_HI 0x29
#define BNO055_I2C_ADDR_LO 0x28
#define BNO055_I2C_ADDR BNO055_I2C_ADDR_LO

#define BNO055_READ_TIMEOUT 100
#define BNO055_WRITE_TIMEOUT 10

#define ERROR_WRITE_SUCCESS 0x01 // Everything working as expected
#define ERROR_WRITE_FAIL \
  0x03                              // Check connection, protocol settings and operation more of BNO055
#define ERROR_REGMAP_INV_ADDR 0x04  // Invalid register address
#define ERROR_REGMAP_WRITE_DIS 0x05 // Register is read-only
#define ERROR_WRONG_START_BYTE 0x06 // Check if the first byte
#define ERROR_BUS_OVERRUN_ERR \
  0x07 // Resend the command, BNO055 was not able to clear the receive buffer
#define ERROR_MAX_LEN_ERR \
  0x08                         // Split the command, max fire size can be up to 128 bytes
#define ERROR_MIN_LEN_ERR 0x09 // Min length of data is less than 1
#define ERROR_RECV_CHAR_TIMEOUT \
  0x0A // Decrease the waiting time between sending of two bytes of one frame

#define REG_WRITE 0x00
#define REG_READ 0x01

/*Definition with the registers of the sensor */
// Page 0
#define BNO055_ID (0xA0)
#define BNO055_CHIP_ID 0x00       // value: 0xA0
#define BNO055_ACC_ID 0x01        // value: 0xFB
#define BNO055_MAG_ID 0x02        // value: 0x32
#define BNO055_GYRO_ID 0x03       // value: 0x0F
#define BNO055_SW_REV_ID_LSB 0x04 // value: 0x08
#define BNO055_SW_REV_ID_MSB 0x05 // value: 0x03
#define BNO055_BL_REV_ID 0x06     // N/A
#define BNO055_PAGE_ID 0x07
#define BNO055_ACC_DATA_X_LSB 0x08
#define BNO055_ACC_DATA_X_MSB 0x09
#define BNO055_ACC_DATA_Y_LSB 0x0A
#define BNO055_ACC_DATA_Y_MSB 0x0B
#define BNO055_ACC_DATA_Z_LSB 0x0C
#define BNO055_ACC_DATA_Z_MSB 0x0D
#define BNO055_MAG_DATA_X_LSB 0x0E
#define BNO055_MAG_DATA_X_MSB 0x0F
#define BNO055_MAG_DATA_Y_LSB 0x10
#define BNO055_MAG_DATA_Y_MSB 0x11
#define BNO055_MAG_DATA_Z_LSB 0x12
#define BNO055_MAG_DATA_Z_MSB 0x13
#define BNO055_GYR_DATA_X_LSB 0x14
#define BNO055_GYR_DATA_X_MSB 0x15
#define BNO055_GYR_DATA_Y_LSB 0x16
#define BNO055_GYR_DATA_Y_MSB 0x17
#define BNO055_GYR_DATA_Z_LSB 0x18
#define BNO055_GYR_DATA_Z_MSB 0x19
#define BNO055_EUL_HEADING_LSB 0x1A
#define BNO055_EUL_HEADING_MSB 0x1B
#define BNO055_EUL_ROLL_LSB 0x1C
#define BNO055_EUL_ROLL_MSB 0x1D
#define BNO055_EUL_PITCH_LSB 0x1E
#define BNO055_EUL_PITCH_MSB 0x1F
#define BNO055_QUA_DATA_W_LSB 0x20
#define BNO055_QUA_DATA_W_MSB 0x21
#define BNO055_QUA_DATA_X_LSB 0x22
#define BNO055_QUA_DATA_X_MSB 0x23
#define BNO055_QUA_DATA_Y_LSB 0x24
#define BNO055_QUA_DATA_Y_MSB 0x25
#define BNO055_QUA_DATA_Z_LSB 0x26
#define BNO055_QUA_DATA_Z_MSB 0x27
#define BNO055_LIA_DATA_X_LSB 0x28
#define BNO055_LIA_DATA_X_MSB 0x29
#define BNO055_LIA_DATA_Y_LSB 0x2A
#define BNO055_LIA_DATA_Y_MSB 0x2B
#define BNO055_LIA_DATA_Z_LSB 0x2C
#define BNO055_LIA_DATA_Z_MSB 0x2D
#define BNO055_GRV_DATA_X_LSB 0x2E
#define BNO055_GRV_DATA_X_MSB 0x2F
#define BNO055_GRV_DATA_Y_LSB 0x30
#define BNO055_GRV_DATA_Y_MSB 0x31
#define BNO055_GRV_DATA_Z_LSB 0x32
#define BNO055_GRV_DATA_Z_MSB 0x33
#define BNO055_TEMP 0x34
#define BNO055_CALIB_STAT 0x35
#define BNO055_ST_RESULT 0x36
#define BNO055_INT_STATUS 0x37
#define BNO055_SYS_CLK_STATUS 0x38
#define BNO055_SYS_STATUS 0x39
#define BNO055_SYS_ERR 0x3A
#define BNO055_UNIT_SEL 0x3B
#define BNO055_OPR_MODE 0x3D
#define BNO055_PWR_MODE 0x3E
#define BNO055_SYS_TRIGGER 0x3F
#define BNO055_TEMP_SOURCE 0x40
#define BNO055_AXIS_MAP_CONFIG 0x41
#define BNO055_AXIS_MAP_SIGN 0x42
#define BNO055_ACC_OFFSET_X_LSB 0x55
#define BNO055_ACC_OFFSET_X_MSB 0x56
#define BNO055_ACC_OFFSET_Y_LSB 0x57
#define BNO055_ACC_OFFSET_Y_MSB 0x58
#define BNO055_ACC_OFFSET_Z_LSB 0x59
#define BNO055_ACC_OFFSET_Z_MSB 0x5A
#define BNO055_MAG_OFFSET_X_LSB 0x5B
#define BNO055_MAG_OFFSET_X_MSB 0x5C
#define BNO055_MAG_OFFSET_Y_LSB 0x5D
#define BNO055_MAG_OFFSET_Y_MSB 0x5E
#define BNO055_MAG_OFFSET_Z_LSB 0x5F
#define BNO055_MAG_OFFSET_Z_MSB 0x60
#define BNO055_GYR_OFFSET_X_LSB 0x61
#define BNO055_GYR_OFFSET_X_MSB 0x62
#define BNO055_GYR_OFFSET_Y_LSB 0x63
#define BNO055_GYR_OFFSET_Y_MSB 0x64
#define BNO055_GYR_OFFSET_Z_LSB 0x65
#define BNO055_GYR_OFFSET_Z_MSB 0x66
#define BNO055_ACC_RADIUS_LSB 0x67
#define BNO055_ACC_RADIUS_MSB 0x68
#define BNO055_MAG_RADIUS_LSB 0x69
#define BNO055_MAG_RADIUS_MSB 0x6A
//
// BNO055 Page 1
#define BNO055_PAGE_ID 0x07
#define BNO055_ACC_CONFIG 0x08
#define BNO055_MAG_CONFIG 0x09
#define BNO055_GYRO_CONFIG_0 0x0A
#define BNO055_GYRO_CONFIG_1 0x0B
#define BNO055_ACC_SLEEP_CONFIG 0x0C
#define BNO055_GYR_SLEEP_CONFIG 0x0D
#define BNO055_INT_MSK 0x0F
#define BNO055_INT_EN 0x10
#define BNO055_ACC_AM_THRES 0x11
#define BNO055_ACC_INT_SETTINGS 0x12
#define BNO055_ACC_HG_DURATION 0x13
#define BNO055_ACC_HG_THRESH 0x14
#define BNO055_ACC_NM_THRESH 0x15
#define BNO055_ACC_NM_SET 0x16
#define BNO055_GYR_INT_SETTINGS 0x17
#define BNO055_GYR_HR_X_SET 0x18
#define BNO055_GYR_DUR_X 0x19
#define BNO055_GYR_HR_Y_SET 0x1A
#define BNO055_GYR_DUR_Y 0x1B
#define BNO055_GYR_HR_Z_SET 0x1C
#define BNO055_GYR_DUR_Z 0x1D
#define BNO055_GYR_AM_THRESH 0x1E
#define BNO055_GYR_AM_SET 0x1F

  /**
   * @brief enums the possibles system status of the device
   * 
   */
  enum bno055_system_status_t
  {

    BNO055_SYSTEM_STATUS_IDLE = 0x00,
    BNO055_SYSTEM_STATUS_SYSTEM_ERROR = 0x01,
    BNO055_SYSTEM_STATUS_INITIALIZING_PERIPHERALS = 0x02,
    BNO055_SYSTEM_STATUS_SYSTEM_INITIALIZATION = 0x03,
    BNO055_SYSTEM_STATUS_EXECUTING_SELF_TEST = 0x04,
    BNO055_SYSTEM_STATUS_FUSION_ALGO_RUNNING = 0x05,
    BNO055_SYSTEM_STATUS_FUSION_ALOG_NOT_RUNNING = 0x06
  };

  /**
   * @brief enums the different operation modes supported by the device
   * 
   */
  typedef enum
  { // BNO-55 operation modes
    BNO055_OPERATION_MODE_CONFIG = 0x00,
    // Sensor Mode
    BNO055_OPERATION_MODE_ACCONLY,
    BNO055_OPERATION_MODE_MAGONLY,
    BNO055_OPERATION_MODE_GYRONLY,
    BNO055_OPERATION_MODE_ACCMAG,
    BNO055_OPERATION_MODE_ACCGYRO,
    BNO055_OPERATION_MODE_MAGGYRO,
    BNO055_OPERATION_MODE_AMG, // 0x07

    // Fusion Mode
    BNO055_OPERATION_MODE_IMU,
    BNO055_OPERATION_MODE_COMPASS,
    BNO055_OPERATION_MODE_M4G,
    BNO055_OPERATION_MODE_NDOF_FMC_OFF,
    BNO055_OPERATION_MODE_NDOF // 0x0C
  } bno055_opmode_t;

  /**
   * @brief structure for saving the state of the test getting from the device
   * 
   */
  typedef struct
  {
    uint8_t mcuState;
    uint8_t gyrState;
    uint8_t magState;
    uint8_t accState;
  } bno055_self_test_result_t;

  /**
   * @brief structure for storing the state of the calibration getting from the device
   * 
   */
  typedef struct
  {
    uint8_t sys;
    uint8_t gyro;
    uint8_t mag;
    uint8_t accel;
  } bno055_calibration_state_t;

  /**
   * @brief structure for storing three-dimensional vectors of int16
   * 
   */
  typedef struct
  {
    int16_t x;
    int16_t y;
    int16_t z;
  } bno055_vector_xyz_int16_t;

  /**
   * @brief structure to interact with the calibration offset
   * 
   */
  typedef struct
  {
    bno055_vector_xyz_int16_t gyro;
    bno055_vector_xyz_int16_t mag;
    bno055_vector_xyz_int16_t accel;
  } bno055_calibration_offset_t;

  typedef struct
  {
    uint16_t mag;
    uint16_t accel;
  } bno055_calibration_radius_t;

  typedef struct
  {
    bno055_calibration_offset_t offset;
    bno055_calibration_radius_t radius;
  } bno055_calibration_data_t;

  /**
   * @brief structur for 4-dimensional vectors
   * 
   */
  typedef struct
  {
    double w;
    double x;
    double y;
    double z;
  } bno055_vector_t;

  /**
   * @brief structur for remapping the axis of the device
   * 
   */
  typedef struct
  {
    uint8_t x;
    uint8_t x_sign;
    uint8_t y;
    uint8_t y_sign;
    uint8_t z;
    uint8_t z_sign;
  } bno055_axis_map_t;

  /**
   * @brief enum to identify the kind of vector to be getting from the communication process
   * 
   */
  typedef enum
  {
    BNO055_VECTOR_ACCELEROMETER = 0x08, // Default: m/s²
    BNO055_VECTOR_MAGNETOMETER = 0x0E,  // Default: uT
    BNO055_VECTOR_GYROSCOPE = 0x14,     // Default: deg/s
    BNO055_VECTOR_EULER = 0x1A,         // Default: degrees
    BNO055_VECTOR_QUATERNION = 0x20,    // No units
    BNO055_VECTOR_LINEARACCEL = 0x28,   // Default: m/s²
    BNO055_VECTOR_GRAVITY = 0x2E        // Default: m/s²
  } bno055_vector_type_t;

  typedef enum
  {
    BNO055_UNITS_ACCELEROMETER_MS2 = 0b00000000,
    BNO055_UNITS_ACCELEROMETER_MG = 0b00000001,
  } bno055_units_acelerometer;

  typedef enum
  {
    BNO055_UNITS_GYROSCOPE_DPS = 0b00000000,
    BNO055_UNITS_GYROSCOPE_RPS = 0b00000010,
  } bno055_units_gyroscope;

  typedef enum
  {
    BNO055_UNITS_EULER_DEG = 0b00000000,
    BNO055_UNITS_EULER_RAD = 0b00000100,
  } bno055_units_euler;

  typedef enum
  {
    BNO055_UNITS_TEMPERATURE_C = 0b00000000,
    BNO055_UNITS_TEMPERATURE_F = 0b00010000,
  } bno055_units_temperature;

  
  enum bno055_system_error_t
  {
    BNO055_SYSTEM_ERROR_NO_ERROR = 0x00,
    BNO055_SYSTEM_ERROR_PERIPHERAL_INITIALIZATION_ERROR = 0x01,
    BNO055_SYSTEM_ERROR_SYSTEM_INITIALIZATION_ERROR = 0x02,
    BNO055_SYSTEM_ERROR_SELF_TEST_FAILED = 0x03,
    BNO055_SYSTEM_ERROR_REG_MAP_VAL_OUT_OF_RANGE = 0x04,
    BNO055_SYSTEM_ERROR_REG_MAP_ADDR_OUT_OF_RANGE = 0x05,
    BNO055_SYSTEM_ERROR_REG_MAP_WRITE_ERROR = 0x06,
    BNO055_SYSTEM_ERROR_LOW_PWR_MODE_NOT_AVAILABLE_FOR_SELECTED_OPR_MODE = 0x07,
    BNO055_SYSTEM_ERROR_ACCEL_PWR_MODE_NOT_AVAILABLE = 0x08,
    BNO055_SYSTEM_ERROR_FUSION_ALGO_CONF_ERROR = 0x09,
    BNO055_SYSTEM_ERROR_SENSOR_CONF_ERROR = 0x0A
  };

  enum bno055_axis_map_representation_t
  {
    BNO055_AXIS_X = 0x00,
    BNO055_AXIS_Y = 0x01,
    BNO055_AXIS_Z = 0x02
  };

  enum bno055_axis_map_sign_t
  {
    BNO055_AXIS_SIGN_POSITIVE = 0x00,
    BNO055_AXIS_SIGN_NEGATIVE = 0x01
  };

  /**
   * @brief write data in a specific register of the sensor
   * 
   * @param reg the register to write in 
   * @param data to be written
   * @return true if the communication was successful 
   * @return false if the communication failed
   */
  bool bno055_writeData(uint8_t reg, uint8_t data);

  /**
   * @brief read data from a specific register of the sensor
   * 
   * @param reg the register to read from
   * @param data the vector where the data is going to be stored
   * @param len the len of the vector to read
   * @return true if the communication was successful
   * @return false if the communication failed
   */
  bool bno055_readData(uint8_t reg, uint8_t *data, uint8_t len);

  /**
   * @brief function that acts like delay for all the proccess of the library work out to choose between a normal delay or the delay from freertos
   * 
   * @param time the time in ms of delay
   */
  void bno055_delay(int time);

  /**
   * @brief assign the interface of I2C which will be used for communication
   * 
   * @param hi2c_device the interface of I2C which will be used for communication
   */
  void bno055_assignI2C(I2C_HandleTypeDef *hi2c_device);

  /**
   * @brief reset the device
   * 
   */
  void bno055_reset();

  /**
   * @brief get the actual operation mode of the device
   * 
   * @return bno055_opmode_t 
   */
  bno055_opmode_t bno055_getOperationMode();

  /**
   * @brief set the operation mode to be used
   * 
   * @param mode the mode in which the imu will be in 
   */
  void bno055_setOperationMode(bno055_opmode_t mode);

  /**
   * @brief puts the device in configuration mode
   * 
   */
  void bno055_setOperationModeConfig();

  /**
   * @brief puts the device in NDOF mode
   * 
   */
  void bno055_setOperationModeNDOF();

  /**
   * @brief set the units of all the magnitudes 
   * 
   * @param acelU units of the acelerometer
   * @param gyrU  units of the gyroscope
   * @param eulU  units of the euler angles
   * @param temU  units of the temperature
   */
  void bno055_setUnits(bno055_units_acelerometer acelU, bno055_units_gyroscope gyrU,
                       bno055_units_euler eulU, bno055_units_temperature temU);

  void bno055_enableExternalCrystal();
  void bno055_disableExternalCrystal();

  /**
   * @brief initialize the bno055 and puts it in configuration mode.
   * 
   * @return true  if the process was successfull.
   * @return false if the ID doesn't match the bno055 id.
   */
  bool bno055_setup();

  /**
   * @brief checks if the device is connected
   * 
   * @return true the device is connected.
   * @return false the ID doesn't match the bno055 id.
   */
  bool bno055_begin();

  /**
   * @brief return the temperature of the device
   * 
   * @return int8_t temperature of the device
   */
  int8_t bno055_getTemp();

  /**
   * @brief gets the value of the bootloader
   * 
   * @return uint8_t  bootloader value
   */
  uint8_t bno055_getBootloaderRevision();

  /**
   * @brief gets the status of the system
   * 
   * @return uint8_t 
   */
  uint8_t bno055_getSystemStatus();

  /**
   * @brief returns errors in the system
   * 
   * @return uint8_t 
   */
  uint8_t bno055_getSystemError();

  /**
   * @brief returns the units of all the interfaces
   * 
   * @return uint8_t 
   */
  uint8_t bno055_getUnits();

  int16_t bno055_getSWRevision();

  bno055_self_test_result_t bno055_getSelfTestResult();
  bno055_calibration_state_t bno055_getCalibrationState();
  bno055_calibration_data_t bno055_getCalibrationData();
  void bno055_setCalibrationData(bno055_calibration_data_t calData);

  /**
   * @brief this functions return the vectors from all the measurements that you get with this device.
   * 
   * @return bno055_vector_t vector with the measurements.
   */
  bno055_vector_t bno055_getVectorAccelerometer();
  /**
   * @brief this functions return the vectors from all the measurements that you get with this device.
   * 
   * @return bno055_vector_t vector with the measurements.
   */
  bno055_vector_t bno055_getVectorMagnetometer();
  /**
   * @brief this functions return the vectors from all the measurements that you get with this device.
   * 
   * @return bno055_vector_t vector with the measurements.
   */
  bno055_vector_t bno055_getVectorGyroscope();
  /**
   * @brief this functions return the vectors from all the measurements that you get with this device.
   * 
   * @return bno055_vector_t vector with the measurements.
   */
  bno055_vector_t bno055_getVectorEuler();
  /**
   * @brief this functions return the vectors from all the measurements that you get with this device.
   * 
   * @return bno055_vector_t vector with the measurements.
   */
  bno055_vector_t bno055_getVectorLinearAccel();
  /**
   * @brief this functions return the vectors from all the measurements that you get with this device.
   * 
   * @return bno055_vector_t vector with the measurements.
   */
  bno055_vector_t bno055_getVectorGravity();
  /**
   * @brief this functions return the vectors from all the measurements that you get with this device.
   * 
   * @return bno055_vector_t vector with the measurements.
   */
  bno055_vector_t bno055_getVectorQuaternion();

  /**
   * @brief remap the axis of the device
   * 
   * @param axis the structure with the configuration to define the axis of the device.
   */
  void bno055_setAxisMap(bno055_axis_map_t axis);

#define UNITS_ACCELEROMETER BNO055_UNITS_ACCELEROMETER_MS2
#define UNITS_GYROSCOPE BNO055_UNITS_GYROSCOPE_RPS
#define UNITS_TEMPERATURE BNO055_UNITS_TEMPERATURE_C
#define UNITS_EULER BNO055_UNITS_EULER_RAD

#ifdef __cplusplus
}
#endif

#endif // BNO055_H_
