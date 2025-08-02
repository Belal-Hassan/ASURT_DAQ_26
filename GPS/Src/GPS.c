/*
 * GPS.c
 *
 *  Created on: Jul 1, 2025
 *      Author: Belal
 */
#include "GPS.h"

extern SemaphoreHandle_t g_i2c_mutex;
extern daq_task_entry_count_t g_task_entry_count;
extern daq_i2c_dma_devices_t g_i2c_dma_device;
extern bool g_i2c_dma_flags[DAQ_NO_OF_I2C_DMA_DEVICES];
char gps_i2c_buffer[45];
gps_gnrmc_data_t gps_data;
static I2C_HandleTypeDef* gps_hi2c;

/*=================Static functions declerations====================*/
static double my_atof(const char *str)
{
    double result = 0.0;
    double fraction = 1.0;
    int sign = 1;

    while (isdigit(*str))
    {
        result = result * 10.0 + (*str - '0');
        str++;
    }

    if (*str == '.')
    {
        str++;
        while (isdigit(*str))
        {
            fraction /= 10.0;
            result += (*str - '0') * fraction;
            str++;
        }
    }

    return result * sign;
}
static double convertToDegrees(char *rawValue)
{
    double val = my_atof(rawValue);
    int deg = (int)(val / 100.0);
    double min = val - (deg * 100.0);
    return deg + (min / 60.0);
}
void GPS_Init(I2C_HandleTypeDef *hi2c)
{
	gps_hi2c = hi2c;
}
void GPS_ReadGNRMC(I2C_HandleTypeDef *hi2c)
{
	if(g_i2c_dma_device == I2C_DMA_NO_DEVICE)
	{
		g_i2c_dma_device = I2C_DMA_GPS;
		HAL_I2C_Master_Receive_IT(hi2c, GPS_I2C_ADDRESS, gps_i2c_buffer, sizeof(gps_i2c_buffer));
	}
}
void GPS_ParseGNRMC(gps_gnrmc_data_t *data)
{
    char *token = strtok(gps_i2c_buffer, ",");
    int field = 0;
    if (strcmp(token, "$GNRMC") != 0)
            return ;

    while (token != NULL)
    {
        switch (field)
        {
            case 1: strncpy(data->time, token, 10); data->time[10] = '\0'; break;
            case 2: data->status = token[0];
            if (token[0]=='V') {return;}
		    break;
            case 3: data->latitude = convertToDegrees(token); break;
            case 4: data->lat_dir = token[0]; break;
            case 5: data->longitude = convertToDegrees(token); break;
            case 6: data->lon_dir = token[0]; return;
        }
        token = strtok(NULL, ",");
        field++;
    }
}
void GPS_Task(void *pvParameters)
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            GPS_ReadGNRMC(gps_hi2c);
            xSemaphoreGive(g_i2c_mutex);
        }
        if(g_i2c_dma_flags[I2C_DMA_GPS])
        {
        	g_i2c_dma_flags[I2C_DMA_GPS] = 0;
        	GPS_ParseGNRMC(&gps_data);
        	// If valid GPS data received
        	if (gps_data.status == 'A')
        	{
        		daq_can_msg_t can_msg_gps = {0};
        		daq_can_msg_gps_t encoder_msg_gps = {0};
        		if(gps_data.lat_dir == 'E')
        			encoder_msg_gps.latitude = (float)gps_data.latitude;
        		else
        			encoder_msg_gps.latitude = -(float)gps_data.latitude;
        		if(gps_data.lon_dir == 'N')
        			encoder_msg_gps.longitude = (float)gps_data.longitude;
        		else
        			encoder_msg_gps.longitude = -(float)gps_data.longitude;

        		can_msg_gps.id = DAQ_CAN_ID_GPS;
        		can_msg_gps.size = 8;
        		can_msg_gps.data = *((uint64_t*)&encoder_msg_gps);
        		DAQ_CAN_Msg_Enqueue(&can_msg_gps);
        	}
        }
        g_task_entry_count.gps++;
        vTaskDelayUntil(&xLastWakeTime, 8);
    }
}
