# chipCap2 library

## How to configure stm32

```c
//main.c
I2C_HandleTypeDef hi2c1;

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

### Putting in configuration mode

to puts the device in configuration mode this steps has to be done:

 - Configure the I2C peripheral.
 - Define the device variable with the information of the sensor
 - Puts the device in configuration mode using the function ***tf02Pro_sendCommand64***

```c
tf02PRO_t tf02_05 = {.Address = 05, .Hi2c_device = &hi2c1};
tf02Pro_sendCommand64(ENTER_CONFIG_MODE ,tf02_05);
```
### Changing the address
to change the address this steps has to be done:

 - Configure the I2C peripheral.
 - Define the device variable with the information of the sensor
 - Puts the device in configuration mode using the function ***tf02Pro_sendCommand64***
 - Change the address of the device using the function ***tf02Pro_sendCommand32***
  
```c
tf02PRO_t tf02_05 = {.Address = 05, .Hi2c_device = &hi2c1};
tf02Pro_sendCommand64(ENTER_CONFIG_MODE ,tf02_05);
//the param of this command is the new address
tf02Pro_sendCommand32(SET_I2C_ADDRESS ,0x28,tf02_05);
tf02Pro_sendCommand64(EXIT_CONFIG_MODE ,tf02_05);
```
### Getting measurements
To get measurements from the sensors just three steps has to be done:

 - Configure the I2C peripheral.
 - Define the address of the respective sensors and the I2C interface for which the device is going to interact with the microcontroller.
 - Checks the device is connected and read the data from the device

```c
tf02PRO_t tf02_05 = {.Address = 05, .Hi2c_device = &hi2c1};
if (tf02Pro_begin(&tf02_05))
{
  tf02Pro_getDataCM(&tf02_05);
  printf("distance %d cm\t temp: %d °C\t  stren:%d \n ",
        tf02_05.Distance, tf02_05.Temp, tf02_05.Strength);
}
```