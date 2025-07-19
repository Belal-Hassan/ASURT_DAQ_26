/*
 * GPS.c
 *
 *  Created on: Jul 1, 2025
 *      Author: Belal
 */
#include "GPS.h"

extern SemaphoreHandle_t i2c_mutex;
extern daq_i2c_dma_devices_t i2c_dma_device;
extern bool i2c_dma_flags[DAQ_NO_OF_I2C_DMA_DEVICES];
static char gps_i2c_buffer[45];
gps_gnrmc_data_t gps_data;
I2C_HandleTypeDef* gps_hi2c;

void GPS_Init(I2C_HandleTypeDef *hi2c)
{
	gps_hi2c = hi2c;
}
void GPS_Read_GNRMC(I2C_HandleTypeDef *hi2c)
{
	if(i2c_dma_device == I2C_DMA_NO_DEVICE)
	{
		i2c_dma_device = I2C_DMA_GPS;
		HAL_I2C_Master_Receive_IT(hi2c, UBLOX_I2C_ADDRESS, gps_i2c_buffer, sizeof(gps_i2c_buffer));
	}
}

void GPS_Parse_GNRMC(gps_gnrmc_data_t *data) {
    //char temp[120];
    //strcpy(temp, sentence);

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
        if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            GPS_Read_GNRMC(gps_hi2c);
            xSemaphoreGive(i2c_mutex);
        }
        if(i2c_dma_flags[I2C_DMA_GPS])
        {
        	i2c_dma_flags[I2C_DMA_GPS] = 0;
        	GPS_Parse_GNRMC(&gps_data);
        	// If valid GPS data received
        	if (gps_data.status == 'A')
        	{
        		daq_can_msg_t can_msg_gps = {};
        		daq_can_msg_gps_t encoder_msg_gps = {};
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
        vTaskDelayUntil(&xLastWakeTime, 11);
    }
}



/*void GPS_sendgps_data(UART_HandleTypeDef *huart, GNRMC_Data *data) {
    char gps_i2c_buffer[128];
    //HAL_Delay(100);
    sprintf(gps_i2c_buffer, "Time: %s\r\n", data->time);
    sendUART(huart, gps_i2c_buffer);
    //HAL_Delay(100);
    sprintf(gps_i2c_buffer, "Status: %c\r\n", data->status);
    sendUART(huart, gps_i2c_buffer);
    //HAL_Delay(100);
    sprintf(gps_i2c_buffer, "Latitude: %.6f %c\r\n", data->latitude, data->lat_dir);
    sendUART(huart, gps_i2c_buffer);
    //HAL_Delay(100);
    sprintf(gps_i2c_buffer, "Longitude: %.6f %c\r\n", data->longitude, data->lon_dir);
    sendUART(huart, gps_i2c_buffer);
    //HAL_Delay(100);
    sprintf(gps_i2c_buffer, "Speed: %.2f Knots\r\n", data->speed);
    sendUART(huart, gps_i2c_buffer);
   // HAL_Delay(100);
    sprintf(gps_i2c_buffer, "Date: %s\r\n", data->date);
    sendUART(huart, gps_i2c_buffer);
   // HAL_Delay(100);
}*/


/*void sendUART(UART_HandleTypeDef *huart, const char *message) {

    HAL_UART_Transmit(huart, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
}*/











double convertToDegrees(char *rawValue) {
    double val = my_atof(rawValue);
    int deg = (int)(val / 100.0);
    double min = val - (deg * 100.0);
    return deg + (min / 60.0);
}

double my_atof(const char *str) {
    double result = 0.0;
    double fraction = 1.0;
    int sign = 1;

    while (isdigit(*str)) {
        result = result * 10.0 + (*str - '0');
        str++;
    }

    if (*str == '.') {
        str++;
        while (isdigit(*str)) {
            fraction /= 10.0;
            result += (*str - '0') * fraction;
            str++;
        }
    }

    return result * sign;
}
