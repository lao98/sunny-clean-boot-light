/*
 * Developer: Bud Ryerson https://github.com/budryerson/TFMini-Plus-I2C/
 * Date:      03 SEP 2020
 * Version:   1.5.0
 * Described: Arduino Library for the Benewake TFMini-Plus Lidar sensor
 *            configured for the I2C interface
 * 
 * Developer Update: Jeison Estiven Garcia @ Robotics 4.0 S.A.S @SunnyApp S.A.S
 * Date:      2021
 * Described: STM32 Library for the Benewake TF02-PRO Lidar sensor
 *            configured for the I2C interface
 */
#include "include/tf02Pro.h"

bool tf02Pro_begin(tf02PRO_t *device){
    HAL_StatusTypeDef result;
    result = HAL_I2C_IsDeviceReady(device->Hi2c_device, device->Address << 1, 2, 2);
    if (result == HAL_OK)
    { // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT
    return true;
    }
    return false;
}

bool tf02Pro_getData(tf02PRO_t *device, uint32_t cmnd)
{
   

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 0 - Command device to ready distance data 
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if (tf02Pro_sendCommand32(cmnd, 0, device) != true)
        return false;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 1 - Get data from the device.
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Request one data-frame from the slave device address
    // and close the I2C interface.
    uint8_t frame[TF_FRAME_SIZE];
    memset(frame, 0, TF_FRAME_SIZE);
    tf02Pro_readData(device, &frame[0], TF_FRAME_SIZE);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 2 - Perform a checksum test.
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    int chkSum = 0;
    // Add together all bytes but the last.
    frame[0] = 0x59;
    for (uint8_t i = 0; i < (TF_FRAME_SIZE - 1); i++)
        chkSum += frame[i];
    //  If the low order byte does not equal the last byte...
    if ((uint8_t)chkSum != frame[TF_FRAME_SIZE - 1])
    {
#if PRINTERROR
        printf("CHECKSUM= %x\n", (uint8_t)chkSum);
        printf("CHECKSUM FAILED \n");
#endif
        return false; // and return "false."
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 3 - Interpret the frame data
    //          and if okay, then go home
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    device->Distance = frame[2] + (frame[3] << 8);
    device->Strength = frame[4] + (frame[5] << 8);
    device->Temp = frame[6] + (frame[7] << 8); // Clear the 'chkSum' variable declared in 'TFMPI2C.h'
    device->Temp = (device->Temp >> 3) - 256;

    return true;
}

bool tf02Pro_getDataCM(tf02PRO_t *device)
{
    return tf02Pro_getData(device, I2C_FORMAT_CM);
}
bool tf02Pro_getDataMM(tf02PRO_t *device)
{
    return tf02Pro_getData(device, I2C_FORMAT_MM);
}

// = = = = =  SEND A COMMAND TO THE DEVICE  = = = = = = = = = =0
//
// Create a proper command byte array, send the command,
// get a response, and return the status.
bool tf02Pro_sendCommand64(uint64_t cmnd, tf02PRO_t *device)
{
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 1 - Build the command data to send to the device
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    uint8_t cmndData[TF_COMMAND_MAX];
    // Clear the send command data array.
    memset(cmndData, 0, TF_COMMAND_MAX);
    memcpy(&cmndData[0], &cmnd, TF_COMMAND_MAX); // Copy 8 bytes of data
    uint8_t replyLen = TF_COMMAND_MAX;    // Save the first byte as reply length.
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 2 - Send the command data array to the device
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Begin transmission to the I2C slave device
    tf02Pro_writeData(device, &cmndData[0], TF_COMMAND_MAX);
    tf02Pro_delay(100);
    // + + + + + + + + + + + + + + + + + + + + + + + + +

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 3 - Get command reply data back from the device.
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    // Request reply data from the device
    uint8_t reply[replyLen];
    tf02Pro_readData(device, &reply[0], replyLen);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 4 - Interpret different command responses.
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    int sum = 0;
    if (cmnd == GET_FIRMWARE_VERSION)
    {
        for (int i = 0; i < 5; i++)
        {
            if (cmndData[i] == reply[i])
            {
                sum++;
            }
        }
        if (sum == 5)
        {
            printf("the firmware version is %d.%d %d.%d %d.%d \n", reply[5], reply[5], reply[6], reply[6], reply[7], reply[7]);
        }
        else
        {
#if PRINTERROR
            printf("Error getting firmware version \n");
#endif
            return false;
        }
    }
    else
    {
        for (int i = 0; i < replyLen; i++)
        {
            if (cmndData[i] == reply[i])
            {
                sum++;
            }
        }
        if (sum == replyLen)
        {
#if PRINTERROR
            printf("Success \n ");
#endif
        }
        else
        {
#if PRINTERROR
            printf("Fail \n");
#endif
            return false;
        }
    }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 6 - Go home
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    return true;
}
// = = = = =  SEND A COMMAND TO THE DEVICE  = = = = = = = = = =0
//
// Create a proper command byte array, send the command,
// get a response, and return the status.
bool tf02Pro_sendCommand32(uint32_t cmnd, uint32_t param, tf02PRO_t *device)
{
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 1 - Build the command data to send to the device
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    uint8_t cmndLen = (cmnd & 0x0000FF00) >> 8; // Save the second byte as command length.
    uint8_t cmndData[cmndLen];
    // Clear the send command data array.
    memset(cmndData, 0, cmndLen);
    memcpy(&cmndData[0], &cmnd, 4); // Copy 4 bytes of data: reply length,
                                       // command length, command number and
                                       // a one byte parameter, all encoded as
                                       // a 32 bit unsigned integer.

    uint8_t replyLen = cmndData[0]; // Save the first byte as reply length.
    cmndData[0] = 0x5A;             // Set the first byte to the header character.

    if (cmnd == SET_FRAME_RATE) // If the command is to Set Frame Rate...
    {
        memcpy(&cmndData[3], &param, 2); // add the 2 byte Frame Rate parameter.
    }
    else if (cmnd == SET_BAUD_RATE) // If the command is to Set Baud Rate...
    {
        memcpy(&cmndData[3], &param, 4); // add the 3 byte Baud Rate parameter.
    }
    else if (cmnd == SET_I2C_ADDRESS) // If the command to set I2C address...
    {
        memcpy(&cmndData[3], &param, 1); // copy the 1 byte Address parameter.
    }

    // Create a checksum byte for the command data array.
   
    int chkSum = 0;
    // Add together all bytes but the last...
    for (uint8_t i = 0; i < (cmndLen - 1); i++)
        chkSum += cmndData[i];
    if (!((cmnd == SET_FRAME_RATE) ||
          (cmnd == SET_BAUD_RATE) ||
          (cmnd == SET_I2C_ADDRESS)))
    {
        // and save it as the last byte of command data.
        cmndData[cmndLen - 1] = (uint8_t)chkSum;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 2 - Send the command data array to the device
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Begin transmission to the I2C slave device
    tf02Pro_writeData(device, &cmndData[0], cmndLen);

    // + + + + + + + + + + + + + + + + + + + + + + + + +
    // If no reply data expected, then go home. Otherwise,
    // wait for device to process the command and continue.
    if (replyLen == 0)
        return true;
    else
        tf02Pro_delay(100);
    // + + + + + + + + + + + + + + + + + + + + + + + + +

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 3 - Get command reply data back from the device.
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    //  An I2C address change will take effect immediately
    //  so use the new 'param' address for the reply.
    if (cmnd == SET_I2C_ADDRESS)
        device->Address = (uint8_t)param;

    // Request reply data from the device
    uint8_t reply[replyLen];
    tf02Pro_readData(device, &reply[0], replyLen);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 4 - Perform a checksum test.
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Clear the 'chkSum' variable declared in 'TFMPI2C.h'
    chkSum = 0;
    // Add together all bytes but the last...
    for (uint8_t i = 0; i < (replyLen - 1); i++)
        chkSum += reply[i];
    // If the low order byte of the Sum does not equal the last byte...
    if (reply[replyLen - 1] != (uint8_t)chkSum)
    {
#if PRINTERROR
        printf("Checksum no coincide \n"); // then set error...
#endif
        return false; // and return "false."
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 5 - Interpret different command responses.
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if (cmnd == FACTORY_RESET)
    {
        if (reply[3] == 1) // If PASS/FAIL byte not zero ...
        {
#if PRINTERROR
            printf("Reinicio de fabrica fallo \n");
#endif
            return false; // and return 'false'.
        }
    }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Step 6 - Go home
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    return true;
}

void tf02Pro_delay(int time)
{
#ifdef FREERTOS_ENABLED
    osDelay(time);
#else
    HAL_Delay(time);
#endif
}

bool tf02Pro_writeData(tf02PRO_t *device, uint8_t *command, uint8_t len)
{
    uint8_t status;
    status = HAL_I2C_Master_Transmit(device->Hi2c_device, device->Address << 1,
                                     command, len, 10);
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

bool tf02Pro_readData(tf02PRO_t *device, uint8_t *data, uint8_t len)
{
    uint8_t status;
    status = HAL_I2C_Master_Receive(device->Hi2c_device, device->Address << 1, data, len,
                                    10);

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
