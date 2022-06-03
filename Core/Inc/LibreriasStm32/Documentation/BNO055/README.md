# bno055 library

## How to configure stm32

```c
//main.c

static void MX_I2C1_Init(void)
{

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
    Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */
}
```

```c
//stm32f4xx_hal_msp.c
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hi2c->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }

}
/**
* @brief I2C MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hi2c: I2C handle pointer
* @retval None
*/
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
  if(hi2c->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }

}
```
## How to use

### Calibration and initialization

the process of initialization consist of the assignment of the interface I2C,definition of the units, realize the begin and the setup, define the operation mode and by last pass the values of a previous calibration or calibrate to get new values.

the modules would be calibrated once the imu return 3 for the state of calibration 

```c
//bno055.h
#define UNITS_ACCELEROMETER BNO055_UNITS_ACCELEROMETER_MS2
#define UNITS_GYROSCOPE BNO055_UNITS_GYROSCOPE_RPS
#define UNITS_TEMPERATURE BNO055_UNITS_TEMPERATURE_C
#define UNITS_EULER BNO055_UNITS_EULER_RAD

//main.c
    bno055_assignI2C(&hi2c1);
	if (bno055_begin())
	{
		if (bno055_setup())
		{
            //set the unit system for the different units
            bno055_setUnits(UNITS_ACCELEROMETER, UNITS_GYROSCOPE, UNITS_EULER, UNITS_TEMPERATURE);
            
            bno055_vector_t v;
			bno055_calibration_state_t cal;
			bno055_calibration_data_t calData;

            //give the values of calibration if you have fom previous process
			calData.offset.accel.x = -23;
			calData.offset.accel.y = 1;
			calData.offset.accel.z = -43;
			calData.offset.gyro.x = -3;
			calData.offset.gyro.y = -3;
			calData.offset.gyro.z = 0;
			calData.offset.mag.x = 0; //-24;
			calData.offset.mag.y = 0; //45;
			calData.offset.mag.z = 0; //315;
			calData.radius.accel = 1000;
			calData.radius.mag = 480;
			
			bno055_setCalibrationData(calData);
			bno055_setOperationMode(BNO055_OPERATION_MODE_IMU);
			
            // wait until the state of the modules is in 3 which means that the calibration is done
			while (!(cal.gyro == 3 && cal.accel == 3))
			{
				//get the values of orientation in vector Euler format to verify progress
                v = bno055_getVectorEuler();
				printf("Heading: %.2f Roll: %.2f Pitch: %.2f\r\n", v.x, v.y, v.z);

                //check the state of the modules 
				cal = bno055_getCalibrationState();
				printf("calibración : a=%d\t g=%d\t m=%d\t s=%d\t\r\n",
					   cal.accel, cal.gyro, cal.mag, cal.sys);
				osDelay(100);
			}

            //confirm that the calibration was completed successfully and get the values resulting from the calibration
			cal = bno055_getCalibrationState();
			printf("calibración : a=%d\t g=%d\t m=%d\t s=%d\t\r\n",
				   cal.accel, cal.gyro, cal.mag, cal.sys);
			bno055_enableExternalCrystal();
			osDelay(1000);
			calData = bno055_getCalibrationData();
			cal = bno055_getCalibrationState();
			printf("...............calibration data................\r\n");
			printf("calibración aceleremetro: x=%d,y=%d,z=%d\r\n",
				   calData.offset.accel.x, calData.offset.accel.y, calData.offset.accel.z);
			printf("calibración giroscopio:  x=%d,y=%d,z=%d\r\n",
				   calData.offset.gyro.x, calData.offset.gyro.y, calData.offset.gyro.z);
			printf("calibración magnetometro:x=%d,y=%d,z=%d\r\n",
				   calData.offset.mag.x, calData.offset.mag.y, calData.offset.mag.z);
			printf("radius aceleremetro: %d\r\n", calData.radius.accel);
			printf("radius magnetometro: %d\r\n", calData.radius.mag);
			printf("calibración : a=%d\t g=%d\t m=%d\t s=%d\t\r\n",
				   cal.accel, cal.gyro, cal.mag, cal.sys);
		}
	}
```
### Get values

```c
    if (bno055_begin())
    {
        
        v = bno055_getVectorGyroscope();
        msgImu.angular_velocity.x = v.x;
        msgImu.angular_velocity.y = v.y;
        msgImu.angular_velocity.z = v.z;
        v = bno055_getVectorLinearAccel();
        msgImu.linear_acceleration.x = v.x;
        msgImu.linear_acceleration.y = v.y;
        msgImu.linear_acceleration.z = v.z;
        v = bno055_getVectorQuaternion();
        msgImu.orientation.w = v.w;
        msgImu.orientation.x = v.x;
        msgImu.orientation.y = v.y;
        msgImu.orientation.z = v.z;
        printf("w: %.4f x: %.4f y: %.4f z:%.4f\r\n", v.w, v.x, v.y, v.z);
    }
```