# MDC100USB

## How to configure stm32
This library has been designed to work with freeRTOS, and is designed to work specially with hardware for the same reason, because the idea is that the communication doesn't take all the cpu time. Having this in mind the library work with DMA (Direct memory access) and hardware interruption, specifically the idle line interrupt that shoot up at event when the uart line enters in a idle state that indicates that the sbus block of data has been received completely.

```c
//main.c
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;
UART_HandleTypeDef huart6;

/* Definitions for uartRX_Mutex */
osMutexId_t uartRX_MutexHandle;
const osMutexAttr_t uartRX_Mutex_attributes = {
    .name = "uartRX_Mutex"};
/* Definitions for uartTX_Mutex */
osSemaphoreId_t uartTX_MutexHandle;
const osSemaphoreAttr_t uartTX_Mutex_attributes = {
    .name = "uartTX_Mutex"};
/* Definitions for uartIdle_sem */
osSemaphoreId_t uartIdle_semHandle;
const osSemaphoreAttr_t uartIdle_sem_attributes = {
    .name = "uartIdle_sem"};

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured MX_LWIP_Init */
  MX_DMA_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
  /* Init scheduler */
  osKernelInitialize();

  uartRX_MutexHandle = osMutexNew(&uartRX_Mutex_attributes);

  /* creation of uartTX_Mutex */
  uartTX_MutexHandle = osSemaphoreNew(1, 1, &uartTX_Mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of uartIdle_sem */
  uartIdle_semHandle = osSemaphoreNew(1, 1, &uartIdle_sem_attributes);

  /* Create the thread(s) */
  /* creation of initTask */
  initTaskHandle = osThreadNew(initTaskFunction, NULL, &initTask_attributes);

  /*Enable the idle interrupt of the uart*/
__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    ......
    .....
    ...
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

//-------INTERRUPTS
void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    if (USART6 == huart->Instance) //Determine whether it is serial port 1
    {
        if (RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) //Judging whether it is idle interruption
        {
            __HAL_UART_CLEAR_IDLEFLAG(huart); //Clear idle interrupt sign (otherwise it will continue to enter interrupt)                       //Call interrupt handler
            osSemaphoreRelease(uartIdle_semHandle);
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    osSemaphoreRelease(uartTX_MutexHandle);
}

//-------CONFIGURATION INTERFACES
/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate =38400;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */
}

static void MX_DMA_Init(void)
{
    /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
}
```

```c
//stm32f4xx_hal_msp.c

extern DMA_HandleTypeDef hdma_usart6_rx;

extern DMA_HandleTypeDef hdma_usart6_tx;
/**
  * Initializes the Global MSP.
  */
/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  /* USER CODE BEGIN USART6_MspInit 0 */

    /* USER CODE END USART6_MspInit 0 */
    __HAL_RCC_USART6_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART6 GPIO Configuration
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USART6 DMA Init */
    /* USART6_RX Init */
    hdma_usart6_rx.Instance = DMA2_Stream1;
    hdma_usart6_rx.Init.Channel = DMA_CHANNEL_5;
    hdma_usart6_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart6_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart6_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart6_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart6_rx.Init.Mode = DMA_NORMAL;
    hdma_usart6_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart6_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart6_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(huart, hdmarx, hdma_usart6_rx);

    /* USART6_TX Init */
    hdma_usart6_tx.Instance = DMA2_Stream6;
    hdma_usart6_tx.Init.Channel = DMA_CHANNEL_5;
    hdma_usart6_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart6_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart6_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart6_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart6_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart6_tx.Init.Mode = DMA_NORMAL;
    hdma_usart6_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart6_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart6_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(huart, hdmatx, hdma_usart6_tx);

    /* USART6 interrupt Init */
    HAL_NVIC_SetPriority(USART6_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
  
}

/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  
  /* USER CODE BEGIN USART6_MspDeInit 0 */

  /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();
  
    /**USART6 GPIO Configuration    
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6|GPIO_PIN_7);

  /* USER CODE BEGIN USART6_MspDeInit 1 */

  /* USER CODE END USART6_MspDeInit 1 */
  }

}
```
## How to use

With the hardware, interrupts, semaphores, mutexes and the freeRTOS completely configured in a task or a timer callback of the RTOS, you can set and get the configuration of the driver and the speed of the motor.

```c

void DriverTaskFunction(void *argument)
{
  mdc100_t driver = {.huart = &huart6, .hdma_usart_rx = &hdma_usart6_rx, .semaphore_uartIdle = &uartIdle_semHandle, .mutex_uart_rx = &uartRX_MutexHandle, .mutex_uart_tx = &uartTX_MutexHandle, .speed = 0, .cmd = true};

  while(1){
    mdc100_status_t status = MDC_100_CORRUPT_DATA;
		if (mdc100_checkError(&driver) == MDC_100_OK)
		{
			status = mdc100_getSpeed(&driver, &driver.speed);
		}
		//printf("speed : %d \n",driver.speed);
		double speed[1] = {(double)driver.speed * 1.0};
		if (status == MDC_100_OK && driver.cmd)
		{
			driver.cmd = false;
			mdc100_setSpeed(&driver, driver.cmdSpeed);
		}
    osDelay(50);
  }
}
```