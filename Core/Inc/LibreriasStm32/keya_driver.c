#include "include/keya_driver.h"



keya_status_t status;


extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
char buffer_rec[20]={0,0,0,0,0,0,0,0,0,0,
		         0,0,0,0,0,0,0,0,0,0};
/*status_t begin(keya_t *device)
{

}*/



keya_status_t checkoutReply(char* reply)
{
    if (reply == NULL)
    {
        return keya_driver_incorrect_format;//...
    }
    else
    {    
        char * reply_cp;
        reply_cp=malloc(sizeof(reply[0])*(strlen(reply)-1));
        memcpy(reply_cp, reply, sizeof(reply[0])*(strlen(reply)-1));
        
        if (!strncmp(reply_cp,"-",1))//1
        {
            return keya_driver_unknown_command;
        }
        else if (!strncmp(reply_cp,"+",1))//0
        {
            return keya_driver_ok;
        }
        free(reply_cp);
    }
    return keya_driver_need_convertion;//...
}

keya_status_t get_converted_data(char* reply, int *buffer)
{
    char data[50];
    char reply_cp[50];
    memset(data,0,50);
    memset(reply_cp,0,50);
    int int_data[10]; //storage the numbers that are got 
    char * d=strchr(reply, '='); //save the spliting address
    size_t len=strlen(reply)-(d-reply+1);
    printf("..converted..\n");
    if(d==NULL)return keya_driver_corrupted_data;
    int i=0;

    do
    {    
        keya_status_t state ;
        memcpy(&reply_cp[0],d+1, len);//delete the characters until =
        printf("len=%d,reply_cp[%d]=%s\n",len,i,reply_cp);
        char* dprev=d;
        d= strchr (d+1, ':') ;
        if(d ==NULL)
        {
           len=strlen(dprev+1);
        }else{
           len=d- dprev-1; //number of elements to save
        }
        memcpy(&data[0],&reply_cp[0], len);//storage the number in data
        printf("len=%d,data[%d]=%s\n",len,i,data);
        state = find_out(&data[0],len);
        if(state != keya_driver_ok) return keya_driver_corrupted_data;
        int_data[i]=atoi(data);
        memset(data,0,50);
        memset(reply_cp,0,50);
        i++;
        if(d!=NULL) len=strlen(d)-1;

    }while (d != NULL);
    memcpy(&buffer[0],&int_data[0],sizeof(int)*(i));
    return keya_driver_ok; 
}

keya_status_t send_command(keya_t *device,char *cmd)
{
    char comm[50];
    int len = sprintf(comm, "%s \r", cmd);
    write_keya(device,(uint8_t *)&comm[0], len);
    read_keya(device);
    char reply[device->buffer_len];
    memcpy(reply,device->buffer_reading,device->buffer_len);

    memcpy(buffer_rec,reply,device->buffer_len);

    return checkoutReply(reply);
}

keya_status_t send_command_params_reading(keya_t *device,char* cmd, int* params, int *buffer, size_t size)
{
    char comm[50];
    size_t len;
    switch (size)
    {
    case 1:
        len = sprintf(comm, "%s %d", cmd, params[0]);
        break;
    case 2:
        len = sprintf(comm, "%s %d %d", cmd, params[0],params[1]);
        break;
    default:
        return keya_driver_incorrect_format;
    }
    char command[len];
    memcpy(&command[0],comm,len);
    keya_status_t state = send_command(device,&command[0]);
    if (state == keya_driver_need_convertion)
    {
    	 state=get_converted_data(buffer_rec, buffer);
		 memset(buffer_rec,0,20);
		 return state;
    }
    else
    {
        status = state;
        return state;
    }
    return keya_driver_ok;
}

keya_status_t send_command_params(keya_t *device,char* cmd, int * params,size_t size)
{
    char comm[50];
    size_t len;
    switch (size)
    {
    case 1:
        len = sprintf(comm, "%s %d", cmd, params[0]);
        break;
    case 2:
        len = sprintf(comm, "%s %d %d", cmd, params[0],params[1]);
        break;
    default:
        return keya_driver_incorrect_format;
    }
    char command[len];
    memcpy(&command[0],comm,len);
    keya_status_t state = send_command(device,&command[0]);
    return state;
}

keya_status_t send_command_reading(keya_t *device,char* cmd, int *buffer)
{
    keya_status_t state = send_command(device,cmd);
    if (state == keya_driver_need_convertion)
    {
    	 printf(".....\n");
         state=get_converted_data(buffer_rec, buffer);
         memset(buffer_rec,0,20);
         return state;
    }
    else
    {
        status = state;
        return state;
    }
}


keya_info_t keya_get_info(keya_t*device)
{
	keya_info_t info;

    bool result = keya_driver_ok;
    get_motor_power(device,info.power);
    result = (result | status);
    get_battery_current(device, info. battery_current);
    result = (result | status);
    get_temperature(device, info.temperature);
    result = (result | status);
    get_voltage(device, info.voltage);
    result = (result | status);
    get_fault_idn(device , info.fault_idn);
    result = (result | status);
    get_state_idn(device , info.state_idn);
    result = (result | status);
   //get_lock_state(device , info.lock_state);
    //status = (result | status);


    return info;
}

void  get_speed(keya_t *device , int *speed)
{
   int read_speed[2]={0,0};
   status = send_command_reading(device,READ_ENCODER_SPEED_RPM, &read_speed[0]);// tener en cuenta

   if (status == keya_driver_ok)
    {

       speed[0] = read_speed[1];
       speed[1] = -read_speed[0];
  }
}

keya_status_t get_status()
{
    return status;
}

void get_motor_power(keya_t *device , int *power)
{
    int read_power[2];

    status = send_command_reading(device,READ_POWER_LEVEL_APPLICATION, read_power);
    if (status == keya_driver_ok)
    {
        power[0] = read_power[0];
        power[1] = read_power[1];
    }

}

void get_battery_current(keya_t *device, int *current)
{
    int read_current[2];

    status = send_command_reading(device,READ_MOTOR_CURRENT, read_current);
    if (status == keya_driver_ok)
    {
        current[0] = read_current[0];
        current[1] = read_current[1];
    }

}

void get_temperature(keya_t *device, float *temperature)
{
    int read_temperature[3];

    status = send_command_reading(device,READ_TEMPERATURE, read_temperature);
    if (status == keya_driver_ok)
    {
        temperature[0] = read_temperature[0] * 1.0;
        temperature[1] = read_temperature[1] * 1.0;
        temperature[2] = read_temperature[2] * 1.0;
    }

}

void get_voltage(keya_t *device, float * voltage )
{
    int read_voltage[3];
    status = send_command_reading(device,READ_INTERNAL_VOLTAGE, read_voltage);
    if (status == keya_driver_ok)
    {
        voltage[0] = read_voltage[0] * 0.1;
        voltage[1] = read_voltage[1] * 0.1;
        voltage[2] = read_voltage[2] * 0.001;
    }
}

void  get_fault_idn(keya_t *device,bool* fault)
{
    int read_fault[1];
    status = send_command_reading(device,READ_FAULT_MARKS, read_fault);
    if (status==keya_driver_ok)
    {
        for (int bit = 0; bit < 8; bit++)
        {
            fault[bit] = read_fault[0] >> bit & 1;
        }
    }
}

void get_state_idn(keya_t *device, bool *state)
{
    int *read_state={0};
    status = send_command_reading(device,READ_STATUS_FLAG, read_state);
    if (status==keya_driver_ok)
    {
        for (int bit = 0; bit < 8; bit++)
        {
            state[bit] = read_state[0] >> bit & 1;
        }
    }
}

void  get_lock_state(keya_t *device , bool *lock_state)
{
    int* read_state={0};
    status = send_command_reading(device,READ_STATUS_FLAG, read_state);
    if (status==keya_driver_ok)
    {
        lock_state = read_state[0] & 1;
    }
}


keya_status_t set_speed(keya_t *device,int * velocity_wheels)
{
    int velocity[2];
    velocity_wheels[1] = -velocity_wheels[1];
    velocity[0]=velocity_wheels[0];
    velocity[1]=velocity_wheels[1];
    return send_command_reading(device,SET_MOTOR_SPEED, velocity);// command -> + reading
}

keya_status_t set_soft_start(keya_t *device,int rpm_slope)
{
    int command[2];
    command[0]=1;
    command[1]=rpm_slope*10;
    keya_status_t state = send_command_params(device,SOFT_START, command,2);
    
    if (state != keya_driver_ok)
        return state;
    command[0]=2;
    command[1]=rpm_slope*10;
    return send_command_params(device,SOFT_START, command,2);
}

keya_status_t set_soft_stop(keya_t *device, int rpm_slope)
{
    int command[2];
    command[0]=1;
    command[1]=rpm_slope*10;
    keya_status_t state = send_command_params(device,SOFT_STOP, command,2);
    if (state != keya_driver_ok)
        return state;
    command[0]=1;
    command[1]=rpm_slope*10;
    return send_command_params(device,SOFT_STOP, command,2);
}
// verificar que es una variable  int
keya_status_t find_out(char *data_cp, int len){
    for (int i=0; i<len;i++)
    {
        if(!isdigit(data_cp[i])){
            if(data_cp[i]!= ' ' && data_cp[i]!= '\n' && data_cp[i]!= '\r')
            {
                return keya_driver_corrupted_data;
            }
        }
    }      
    return keya_driver_ok ;  
}

void write_keya(keya_t *device,uint8_t *data, int len)
{

	osMutexAcquire(*device->uartTXsem,osWaitForever);// try to acquire mutex
	HAL_UART_Transmit_IT(device->huart, data, len);
}

bool read_keya(keya_t *device)
{
    osMutexAcquire(*device->uartRXmutex, osWaitForever);// try to acquire mutex
	bool status;
	//receive the data using dma (direct memory access) to avoid excessive consumption
	memset(device->buffer_reading,0,50);
	status=HAL_UART_Receive_DMA(device->huart,device->buffer_reading, 50)==HAL_OK ? 1:0;
	osSemaphoreAcquire(*device->uartRXsem,  (TickType_t)10*portTICK_PERIOD_MS);
	osStatus_t semStatus=osSemaphoreAcquire(*device->uartRXsem, osWaitForever);// (TickType_t)10*portTICK_PERIOD_MS);
	if( semStatus!=osOK || status==false){
		status=false;
	}
	device->buffer_len  = device-> BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);

	HAL_UART_DMAStop(device->huart);
#if DEBUG_KEYA
	printf("R:%s\n",(char *)device->buffer_reading);
#endif
	osSemaphoreRelease(*device->uartRXsem);
	osMutexRelease(*device->uartRXmutex);
	return status;

}
