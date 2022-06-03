# Radiolink R12DS 2.4GHz RC receptor 12 canales SBUS/PWM

- Stronger Anti-Interference: DSSS and FHSS communication technology makes R12DS receiver with stable transmission signal, control distance up to 2.4 miles in the air. (The control range depends on the use environment)
- 12 channels: with standard SBUS/PWM signal output, high resolution ratio of 2048, fast response for all channels

## Sbus
This library communicates with SBUS receivers and servos and is compatible with stm32 microcontrollers.

## Description
SBUS is a bus protocol for receivers to send commands to servos. Unlike PWM, SBUS uses a bus architecture where a single serial line can be connected with up to 16 servos with each receiving a unique command.

The SBUS protocol uses an inverted serial logic with a baud rate of 100000, 8 data bits, even parity, and 2 stop bits. The SBUS packet is 25 bytes long consisting of:
   * Byte[0]: SBUS header, 0x0F
   * Byte[1 -22]: 16 servo channels, 11 bits each
   * Byte[23]
      * Bit 0: channel 17 (0x01)
      * Bit 1: channel 18 (0x02)
      * Bit 2: frame lost (0x04)
      * Bit 3: failsafe activated (0x08)
   * Byte[24]: SBUS footer 0x00

Note that lost frame is indicated when a frame is lost between the transmitter and receiver. Failsafe activation typically requires that many frames are lost in a row and indicates that the receiver has moved into failsafe mode. Packets are sent approximately every 10 ms or 20 ms, depending on the system configuration.

## How to configure stm32
This library has been designed to work with freeRTOS, and is designed to work specially with hardware for the same reason, because the idea is that the communication doesn't take all the cpu time. Having this in mind the library work with DMA (Direct memory access) and hardware interruption, specifically the idle line interrupt that shoot up at event when the uart line enters in a idle state that indicates that the sbus block of data has been received completely.
```c
//main.c
DMA_HandleTypeDef hdma_usart6_rx;
UART_HandleTypeDef huart6;

osMutexId_t uartRX_MutexHandle;
const osMutexAttr_t uartRX_Mutex_attributes = {
    .name = "uartRX_Mutex"};

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

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* creation of uartRX_Mutex */
  uartRX_MutexHandle = osMutexNew(&uartRX_Mutex_attributes);
  /* USER CODE END RTOS_MUTEX */


  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* creation of uartIdle_sem */
  uartIdle_semHandle = osSemaphoreNew(1, 1, &uartIdle_sem_attributes);
  /* USER CODE END RTOS_SEMAPHORES */

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
  huart6.Init.BaudRate = 100000;
  huart6.Init.WordLength = UART_WORDLENGTH_9B;
  huart6.Init.StopBits = UART_STOPBITS_2;
  huart6.Init.Parity = UART_PARITY_EVEN;
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
}
```

```c
//stm32f4xx_hal_msp.c

extern DMA_HandleTypeDef hdma_usart6_rx;

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  if(huart->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspInit 0 */

  /* USER CODE END USART6_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART6 GPIO Configuration
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
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
    hdma_usart6_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_usart6_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart6_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(huart,hdmarx,hdma_usart6_rx);

       /* USART6 interrupt Init */
    HAL_NVIC_SetPriority(USART6_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);

  /* USER CODE BEGIN USART6_MspInit 1 */

  /* USER CODE END USART6_MspInit 1 */
  }

}

/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  
  if(huart->Instance==USART3)
  {
    
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();
  
    /**USART3 GPIO Configuration    
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 DMA DeInit */
    HAL_DMA_DeInit(huart->hdmarx);
    HAL_DMA_DeInit(huart->hdmatx);

    /* USART3 interrupt DeInit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
  else if(huart->Instance==USART6)
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

With the hardware, interrupts, semaphores, mutexes and the freeRTOS completely configured in a task or a timer callback of the RTOS, you can get the sbus data with the function *** r12ds_getChannels(r12ds_t *device)***

```c
void rfTaskFunction(void *argument)
{
  r12ds_t rf = {.huart = &huart6, .hdma_usart_rx = &hdma_usart6_rx, .semaphore_uartIdle = &uartIdle_semHandle, .mutex_uart_rx = &uartRX_MutexHandle};

  while(1){
    r12ds_getChannels(&rf);
    osDelay(50);
  }
}
```
