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
#include "include/bno055.h"
#include <string.h>

//declare the variables for the scale of every magnitude having in count the units of each one.
uint16_t accelScale = (UNITS_ACCELEROMETER == BNO055_UNITS_ACCELEROMETER_MG ? (1) : (100));
uint16_t tempScale = 1;
uint16_t angularRateScale = (UNITS_GYROSCOPE == BNO055_UNITS_GYROSCOPE_DPS ? (16) : (900));
uint16_t eulerScale = (UNITS_EULER == BNO055_UNITS_EULER_DEG ? (16) : (900));
uint16_t magScale = 16;
uint16_t quaScale = (1 << 14); // 2^14

I2C_HandleTypeDef *_bno055_i2c_port;

void bno055_setPage(uint8_t page)
{
  bno055_writeData(BNO055_PAGE_ID, page);
}

bno055_opmode_t bno055_getOperationMode()
{
  bno055_opmode_t mode;
  bno055_readData(BNO055_OPR_MODE, &mode, 1);
  return mode;
}

void bno055_setOperationMode(bno055_opmode_t mode)
{
  bno055_writeData(BNO055_OPR_MODE, mode);
  if (mode == BNO055_OPERATION_MODE_CONFIG)
  {
    bno055_delay(19);
  }
  else
  {
    bno055_delay(7);
  }
}

void bno055_setOperationModeConfig()
{
  bno055_setOperationMode(BNO055_OPERATION_MODE_CONFIG);
}

void bno055_setOperationModeNDOF()
{
  bno055_setOperationMode(BNO055_OPERATION_MODE_NDOF);
}

void bno055_setUnits(bno055_units_acelerometer acelU, bno055_units_gyroscope gyrU,
                     bno055_units_euler eulU, bno055_units_temperature temU)
{
  bno055_writeData(BNO055_UNIT_SEL, acelU | gyrU | eulU | temU);
}

void bno055_setExternalCrystalUse(bool state)
{
  bno055_setPage(0);
  uint8_t tmp = 0;
  bno055_readData(BNO055_SYS_TRIGGER, &tmp, 1);
  tmp |= (state == true) ? 0x80 : 0x0;
  bno055_writeData(BNO055_SYS_TRIGGER, tmp);
  bno055_delay(700);
}

void bno055_enableExternalCrystal() { bno055_setExternalCrystalUse(true); }
void bno055_disableExternalCrystal() { bno055_setExternalCrystalUse(false); }

void bno055_reset()
{
  bno055_writeData(BNO055_SYS_TRIGGER, 0x20);
  bno055_delay(700);
}

int8_t bno055_getTemp()
{
  bno055_setPage(0);
  uint8_t t;
  bno055_readData(BNO055_TEMP, &t, 1);
  return t;
}

bool bno055_setup()
{
  bno055_reset();

  uint8_t id = 0;
  bno055_readData(BNO055_CHIP_ID, &id, 1);
  if (id != BNO055_ID)
  {
    printf("Can't find BNO055, id: 0x%02x. Please check your wiring.\r\n", id);
    return false;
  }
  bno055_setPage(0);
  bno055_writeData(BNO055_SYS_TRIGGER, 0x0);

  // Select BNO055 config mode
  bno055_setOperationModeConfig();
  bno055_delay(10);
  return true;
}

bool bno055_begin(){
    uint8_t id = 0;
    bno055_readData(BNO055_CHIP_ID, &id, 1);
      if (id != BNO055_ID)
      {
        printf("Can't find BNO055, id: 0x%02x. Please check your wiring.\r\n", id);
        return false;
      }
    return true;
}

int16_t bno055_getSWRevision()
{
  bno055_setPage(0);
  uint8_t buffer[2];
  bno055_readData(BNO055_SW_REV_ID_LSB, buffer, 2);
  return (int16_t)((buffer[1] << 8) | buffer[0]);
}

uint8_t bno055_getBootloaderRevision()
{
  bno055_setPage(0);
  uint8_t tmp;
  bno055_readData(BNO055_BL_REV_ID, &tmp, 1);
  return tmp;
}

uint8_t bno055_getSystemStatus()
{
  bno055_setPage(0);
  uint8_t tmp;
  bno055_readData(BNO055_SYS_STATUS, &tmp, 1);
  return tmp;
}

bno055_self_test_result_t bno055_getSelfTestResult()
{
  bno055_setPage(0);
  uint8_t tmp;
  bno055_self_test_result_t res = {
      .mcuState = 0, .gyrState = 0, .magState = 0, .accState = 0};
  bno055_readData(BNO055_ST_RESULT, &tmp, 1);
  res.mcuState = (tmp >> 3) & 0x01;
  res.gyrState = (tmp >> 2) & 0x01;
  res.magState = (tmp >> 1) & 0x01;
  res.accState = (tmp >> 0) & 0x01;
  return res;
}

uint8_t bno055_getSystemError()
{
  bno055_setPage(0);
  uint8_t tmp;
  bno055_readData(BNO055_SYS_ERR, &tmp, 1);
  return tmp;
}

uint8_t bno055_getUnits()
{
  bno055_setPage(0);
  uint8_t tmp;
  bno055_readData(BNO055_UNIT_SEL, &tmp, 1);
  if (tmp & 0b00000001)
  {
    printf("acel: mg \t");
  }
  else
  {
    printf("acel: m/s²\t");
  }
  if (tmp & 0b00000010)
  {
    printf("gyro: Rps \t");
  }
  else
  {
    printf("gyro: Dps \t");
  }
  if (tmp & 0b00000100)
  {
    printf("Euler: Rad \t");
  }
  else
  {
    printf("Euler: Deg \t");
  }
  if (tmp & 0b00010000)
  {
    printf("Temp: °f \n");
  }
  else
  {
    printf("Temp: °c \n");
  }

  printf("AcelSca: %d\t", accelScale);
  printf("GyroSca: %d\t", angularRateScale);
  printf("EulerSca: %d\t", eulerScale);
  return tmp;
}

bno055_calibration_state_t bno055_getCalibrationState()
{
  bno055_setPage(0);
  bno055_calibration_state_t cal = {.sys = 0, .gyro = 0, .mag = 0, .accel = 0};
  uint8_t calState = 0;
  bno055_readData(BNO055_CALIB_STAT, &calState, 1);
  cal.sys = (calState >> 6) & 0x03;
  cal.gyro = (calState >> 4) & 0x03;
  cal.accel = (calState >> 2) & 0x03;
  cal.mag = calState & 0x03;
  return cal;
}

bno055_calibration_data_t bno055_getCalibrationData()
{
  bno055_calibration_data_t calData;
  uint8_t buffer[22];
  bno055_opmode_t operationMode = bno055_getOperationMode();
  bno055_setOperationModeConfig();
  bno055_setPage(0);

  bno055_readData(BNO055_ACC_OFFSET_X_LSB, buffer, 22);

  // Assumes little endian processor
  memcpy(&calData.offset.accel, buffer, 6);
  memcpy(&calData.offset.mag, buffer + 6, 6);
  memcpy(&calData.offset.gyro, buffer + 12, 6);
  memcpy(&calData.radius.accel, buffer + 18, 2);
  memcpy(&calData.radius.mag, buffer + 20, 2);

  bno055_setOperationMode(operationMode);

  return calData;
}

void bno055_setCalibrationData(bno055_calibration_data_t calData)
{
  uint8_t buffer[22];
  bno055_opmode_t operationMode = bno055_getOperationMode();
  bno055_setOperationModeConfig();
  bno055_delay(25);
  bno055_setPage(0);

  // Assumes litle endian processor
  memcpy(buffer, &calData.offset.accel, 6);
  memcpy(buffer + 6, &calData.offset.mag, 6);
  memcpy(buffer + 12, &calData.offset.gyro, 6);
  memcpy(buffer + 18, &calData.radius.accel, 2);
  memcpy(buffer + 20, &calData.radius.mag, 2);

  for (uint8_t i = 0; i < 22; i++)
  {
    // TODO(oliv4945): create multibytes write
    bno055_writeData(BNO055_ACC_OFFSET_X_LSB + i, buffer[i]);
  }

  bno055_setOperationMode(operationMode);
}

bno055_vector_t bno055_getVector(uint8_t vec)
{
  bno055_setPage(0);
  uint8_t buffer[8]={0,0,0,0,0,0,0,0}; // Quaternion need 8 bytes

  if (vec == BNO055_VECTOR_QUATERNION)
    bno055_readData(vec, buffer, 8);
  else
    bno055_readData(vec, buffer, 6);
  double scale = 1;

  if (vec == BNO055_VECTOR_MAGNETOMETER)
  {
    scale = magScale;
  }
  else if (vec == BNO055_VECTOR_ACCELEROMETER ||
           vec == BNO055_VECTOR_LINEARACCEL || vec == BNO055_VECTOR_GRAVITY)
  {
    scale = accelScale;
  }
  else if (vec == BNO055_VECTOR_GYROSCOPE)
  {
    scale = angularRateScale;
  }
  else if (vec == BNO055_VECTOR_EULER)
  {
    scale = eulerScale;
  }
  else if (vec == BNO055_VECTOR_QUATERNION)
  {
    scale = quaScale;
  }

  bno055_vector_t xyz = {.w = 0, .x = 0, .y = 0, .z = 0};
  if (vec == BNO055_VECTOR_QUATERNION)
  {
    xyz.w = (int16_t)((buffer[1] << 8) | buffer[0]) / scale;
    xyz.x = (int16_t)((buffer[3] << 8) | buffer[2]) / scale;
    xyz.y = (int16_t)((buffer[5] << 8) | buffer[4]) / scale;
    xyz.z = (int16_t)((buffer[7] << 8) | buffer[6]) / scale;
  }
  else
  {
    xyz.x = (int16_t)((buffer[1] << 8) | buffer[0]) / scale;
    xyz.y = (int16_t)((buffer[3] << 8) | buffer[2]) / scale;
    xyz.z = (int16_t)((buffer[5] << 8) | buffer[4]) / scale;
  }

  return xyz;
}

bno055_vector_t bno055_getVectorAccelerometer()
{
  return bno055_getVector(BNO055_VECTOR_ACCELEROMETER);
}
bno055_vector_t bno055_getVectorMagnetometer()
{
  return bno055_getVector(BNO055_VECTOR_MAGNETOMETER);
}
bno055_vector_t bno055_getVectorGyroscope()
{
  return bno055_getVector(BNO055_VECTOR_GYROSCOPE);
}
bno055_vector_t bno055_getVectorEuler()
{
  return bno055_getVector(BNO055_VECTOR_EULER);
}
bno055_vector_t bno055_getVectorLinearAccel()
{
  return bno055_getVector(BNO055_VECTOR_LINEARACCEL);
}
bno055_vector_t bno055_getVectorGravity()
{
  return bno055_getVector(BNO055_VECTOR_GRAVITY);
}
bno055_vector_t bno055_getVectorQuaternion()
{
  return bno055_getVector(BNO055_VECTOR_QUATERNION);
}

void bno055_setAxisMap(bno055_axis_map_t axis)
{
  uint8_t axisRemap = (axis.z << 4) | (axis.y << 2) | (axis.x);
  uint8_t axisMapSign = (axis.x_sign << 2) | (axis.y_sign << 1) | (axis.z_sign);
  bno055_writeData(BNO055_AXIS_MAP_CONFIG, axisRemap);
  bno055_writeData(BNO055_AXIS_MAP_SIGN, axisMapSign);
}


void bno055_assignI2C(I2C_HandleTypeDef *hi2c_device)
{
  _bno055_i2c_port = hi2c_device;
}

void bno055_delay(int time)
{
#ifdef FREERTOS_ENABLED
  osDelay(time);
#else
  HAL_Delay(time);
#endif
}

bool bno055_writeData(uint8_t reg, uint8_t data)
{
  uint8_t txdata[2] = {reg, data};
  uint8_t status;
  status = HAL_I2C_Master_Transmit(_bno055_i2c_port, BNO055_I2C_ADDR << 1,
                                   txdata, sizeof(txdata), 10);
  if (status == HAL_OK)
  {
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

  uint32_t error = HAL_I2C_GetError(_bno055_i2c_port);
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

  HAL_I2C_StateTypeDef state = HAL_I2C_GetState(_bno055_i2c_port);
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

bool bno055_readData(uint8_t reg, uint8_t *data, uint8_t len)
{
  uint8_t status;
  status = HAL_I2C_Master_Transmit(_bno055_i2c_port, BNO055_I2C_ADDR << 1, &reg, 1,
                                   10);
  if (status == HAL_OK)
  {
    status = HAL_I2C_Master_Receive(_bno055_i2c_port, BNO055_I2C_ADDR << 1, data, len,
                                    10);
    if (status == HAL_OK)
    {
      return true;
    }
    return false;
  }
  return false;
}
