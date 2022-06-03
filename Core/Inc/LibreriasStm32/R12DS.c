/*
 *
 *
 *  Created on: feb 18, 2022
 *      Author: leo
 */

#include "include/R12DS.h"
extern DMA_HandleTypeDef hdma_usart3_rx;



rf_t rf_={.max_speed_linear=MAX_SPEED_LINEAR , .min_speed_linear=MIN_SPEED_LINEAR, .max_speed_angular=MAX_SPEED_ANGULAR,
.min_speed_angular= MIN_SPEED_ANGULAR , .min_speed_brush=MIN_SPEED_BRUSH ,.upper_limit_channels=UPPER_LIMIT_CHANNELS, .lower_limit_channels= LOWER_LIMIT_CHANNELS,
.zero_channels=ZERO_CHANNELS,.threshold=THRESHOLD};



r9ds_status_t r9ds_getChannels(r9ds_t *device)
{
;
for(int i=0;i<10;i++) device->channels[i]=0;
	if (r9ds_readData(device))
	{
	    for(int i=0;i<26;i++){
		    device->buffer[i]=device->buffer_rf[i+1];
	    }
		if (device->buffer[0] == R12DS_STARTBYTE && device->buffer[24] == R12DS_FOOTERBYTE)
		{

			device->channels[0] = ((device->buffer[1] | device->buffer[2] << 8) & 0x07FF);
			device->channels[1] = ((device->buffer[2] >> 3 | device->buffer[3] << 5) & 0x07FF);
			device->channels[2] = ((device->buffer[3] >> 6 | device->buffer[4] << 2 | device->buffer[5] << 10) & 0x07FF);
			device->channels[3] = ((device->buffer[5] >> 1 | device->buffer[6] << 7) & 0x07FF);
			device->channels[4] = ((device->buffer[6] >> 4 | device->buffer[7] << 4) & 0x07FF);
			device->channels[5] = ((device->buffer[7] >> 7 | device->buffer[8] << 1 |device-> buffer[9] << 9) & 0x07FF);
			device->channels[6] = ((device->buffer[9] >> 2 | device->buffer[10] << 6) & 0x07FF);
			device->channels[7] = ((device->buffer[10] >> 5 | device->buffer[11] << 3) & 0x07FF);
			device->channels[8] = ((device->buffer[12] | device->buffer[13] << 8) & 0x07FF);
			device->channels[9] = ((device->buffer[13] >> 3 | device->buffer[14] << 5) & 0x07FF);

			printf("canal 1 %d\n",device->channels[0]);
			printf("canal 2 %d\n",device->channels[1]);
			printf("canal 3 %d\n",device->channels[2]);
			printf("canal 4 %d\n",device->channels[3]);
			printf("canal 5 %d\n",device->channels[4]);
			printf("canal 6 %d\n",device->channels[5]);
			printf("canal 7 %d\n",device->channels[6]);
			printf("canal 8 %d\n",device->channels[7]);
			printf("canal 9 %d\n",device->channels[8]);
			printf("canal 10 %d\n",device->channels[9]);

            HAL_Delay(100);

			return R12DS_OK;

		}
		else
		{
			if (device->buffer[0] != R12DS_STARTBYTE)
			{
#if R12DS_DEBUG
				//printf("bad header \n");
#endif
				return R12DS_BAD_HEADER;
			}
			if (device->buffer[24] != R12DS_FOOTERBYTE)
			{
#if R12DS_DEBUG
				//printf("bad footer \n");
#endif
				return R12DS_BAD_FOOTER;
			}
			if (((device->buffer[23] >> 3) & 1))
			{
#if R12DS_DEBUG
				//printf("FAILSAFE \n");
#endif
				return R12DS_FAILSAFE_ACTIVATED;
			}
			if (((device->buffer[23] >> 2) & 1))
			{
#if R12DS_DEBUG
				//printf("frame lost \n");
#endif
				return R12DS_FRAME_LOST;
			}
		}
	}
	else
	{
#if R12DS_DEBUG
		printf("timeout \n");
#endif
		return R12DS_RX_TIMEOUT;
	}
	return R12DS_OK;
}







bool r9ds_readData(r9ds_t *device)//
{
	//block the use of the uart 

	memset(device->buffer_rf, 0, 26);
	osMutexAcquire(*device->mutex_uart_rx, osWaitForever);
	//block the semaphore until the actual block of sbus data is received, 
	//the idle interruption of uart will free the semaphore for the nextacquisition.
	osSemaphoreAcquire(*device->semaphore_uartIdle, (TickType_t)30* portTICK_PERIOD_MS);// 30->10
	bool status;
	//blocks the semaphore until the process of reading is the full sbus is completed
	//if can acquire it means that something happen to the connection 
	if (osSemaphoreAcquire(*device->semaphore_uartIdle, (TickType_t)30* portTICK_PERIOD_MS) == osOK)
	{
		
		status = HAL_UART_Receive_DMA(device->huart_r9ds,device->buffer_rf, 26) == HAL_OK ? 1 : 0;

		//checks that the semaphore really was release by the idle interruption
		if (osSemaphoreAcquire(*device->semaphore_uartIdle, (TickType_t)30* portTICK_PERIOD_MS) != osOK || status == false)
		{
			status = false;
		}
		HAL_UART_DMAStop(device->huart_r9ds);
#if R12DS_DEBUG
		printf("R: %s \n",device->buffer_rf);
#endif
	}
	//release the semaphore for the next acquisition
	osSemaphoreRelease(*device->semaphore_uartIdle);
	//release the uart interface
	osMutexRelease(*device->mutex_uart_rx);
	return status;
}


void updateCallback(r9ds_t* device)
    {
        data_t data_rec;
        data_t data_;

        data_rec.onOff.value = convertSwitch(device->channels[CH_ONOFF]);
        data_rec.onOff.change = (data_.onOff.value != data_rec.onOff.value) ? 1 : 0;

        data_rec.brake.value = convertSwitch(device->channels[CH_BRAKE]);
        data_rec.brake.change = (data_.brake.value != data_rec.brake.value) ? 1 : 0;

        data_rec.elecValOnOff.value = convertSwitch(device->channels[CH_ELECVAL_ONOFF]);
        data_rec.elecValOnOff.change=  (data_.elecValOnOff.value != data_rec.elecValOnOff.value) ? 1 : 0;

        data_rec.brushesOnOff.value = convertSwitch(device->channels[CH_BRU_ONOFF]);
        data_rec.brushesOnOff.change = (data_.brushesOnOff.value != data_rec.brushesOnOff.value) ? 1 : 0;

        data_rec.elecVal.value =  convert3Switch(device->channels[CH_ELECVAL]); //data_rec.elecValOnOff.value ? convert3Switch(device->channels[CH_ELECVAL]) : 0;
        data_rec.elecVal.change = (data_.elecVal.value != data_rec.elecVal.value) ? 1 : 0;

        int brushes = convert3Switch(device->channels[CH_BRU]);

        data_rec.brush.value=brushes;
        data_rec.brush.value= (data_.brush.value != data_rec.brush.value) ? 1 : 0;

        data_rec.frontBrushSpeed.value = ((brushes == 1 ||
                                     brushes == 2) &&
                                    data_rec.brushesOnOff.value)
                                       ? convertKnob(device->channels[CH_BRU0_VEL],
                                                     rf_.max_speed_brush, rf_.min_speed_brush)
                                       : 0.0;
                                       
        data_rec.frontBrushSpeed.change = (data_.frontBrushSpeed.value != data_rec.frontBrushSpeed.value) ? 1 : 0;

        data_rec.backBrushSpeed.value = ((brushes == 3 ||
                                    brushes == 2) &&
                                   data_rec.brushesOnOff.value)
                                      ? convertKnob(device->channels[CH_BRU1_VEL],
                                                    rf_.max_speed_brush, rf_.min_speed_brush)
                                      : 0.0;
        data_rec.backBrushSpeed.change = (data_.backBrushSpeed.value != data_rec.backBrushSpeed.value) ? 1 : 0;

        data_rec.robotLinearVelocity.value = convertKnob(device->channels[CH_LIN_VEL], rf_.min_speed_linear, rf_.max_speed_linear);
        data_rec.robotLinearVelocity.change = (data_.robotLinearVelocity.value != data_rec.robotLinearVelocity.value) ? 1 : 0;
        if ( data_rec.robotLinearVelocity.value > 1 ){
        	data_rec.robotLinearVelocity.value=1;
        }
        device->control= data_rec.robotLinearVelocity.value ;
        printf(" value_lineal: %f\n",device->control);

        data_rec.robotAngularVelocity.value = convertKnob(device->channels[CH_ANG_VEL],rf_.min_speed_angular, rf_.max_speed_angular);
        data_rec.robotAngularVelocity.change = (data_.robotAngularVelocity.value != data_rec.robotAngularVelocity.value) ? 1 : 0;

        device->control_w= - data_rec.robotAngularVelocity.value ;
        printf(" value_angular: %f\n",device->control_w);
        //data_ = data_rec;

        //callback_rf_();
        return data_rec;
    }

int convertSwitch(int value)
    {   
        //
		if (value < rf_.upper_limit_channels + rf_.threshold && value > rf_.upper_limit_channels - rf_.threshold)
            return 1;
        if (value < rf_.lower_limit_channels + rf_.threshold && value > rf_.lower_limit_channels - rf_.threshold)
            return 0;
        return -1;
    }
int convert3Switch(int value)
    {
        if (value < rf_.zero_channels + rf_.threshold && value > rf_.zero_channels - rf_.threshold)

            return 2;

        if (value < rf_.upper_limit_channels + rf_.threshold && value > rf_.upper_limit_channels - rf_.threshold)

            return 1;

        if (value < rf_.lower_limit_channels + rf_.threshold && value > rf_.lower_limit_channels - rf_.threshold)

            return 3;
        return -1;
    }

float convertKnob(int value, float upper_lim, float lower_lim)
    {
        if (value < rf_.zero_channels + 20 && value > rf_.zero_channels - 20)
            return 0;


        else if (value > rf_.lower_limit_channels - 50 && value < rf_.upper_limit_channels + 50)
        {
            float m = (float)(upper_lim-lower_lim) / (float)(rf_.upper_limit_channels - rf_.lower_limit_channels);
            float ca=m*(float)(value - rf_.lower_limit_channels) + lower_lim;
            //printf("valor = %f\n",ca);
            return ca;
        }
        return 0;
    }
