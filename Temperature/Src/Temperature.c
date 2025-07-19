/*
 * Temperature.c
 *
 *  Created on: Jul 1, 2025
 *      Author: Belal
 */
#include "Temperature.h"

extern SemaphoreHandle_t i2c_mutex;
float wheel_temps[DAQ_NO_OF_TEMP_SENSORS];
moving_avg_t temp_moving_avgs[DAQ_NO_OF_TEMP_SENSORS];
temp_sensor_data_t temp_sensors_data[DAQ_NO_OF_TEMP_SENSORS];
I2C_HandleTypeDef* temp_hi2c;

void Temp_Init(I2C_HandleTypeDef * hi2c)
{
	temp_hi2c = hi2c;
}
// Function to read temperature from a specific register
void MLX90614_Read_Data(uint8_t sensor_index)
{
		// Read 3 bytes from the specified register
		HAL_I2C_Mem_Read(temp_hi2c, (MLX90614_ADDR + sensor_index * I2C_ADDR_MULTIPLIER) << 1, MLX90614_TOBJ, I2C_MEMADD_SIZE_8BIT, temp_sensors_data[sensor_index].data, 3, HAL_MAX_DELAY);
}
float MLX90614_Process_Temp(uint8_t sensor_index)
{
	// Combine the two data bytes into a 16-bit value
	temp_sensors_data[sensor_index].raw = (temp_sensors_data[sensor_index].data[1] << 8) | temp_sensors_data[sensor_index].data[0];

	// Convert raw value to Celsius (according to datasheet)
	temp_sensors_data[sensor_index].temperature = (temp_sensors_data[sensor_index].raw * 0.02) - 273.15;

	return Moving_Avg(temp_sensors_data[sensor_index].temperature, sensor_index);
}
// Function to read ambient temperature
/*float MLX90614_ReadAmbientTemp(I2C_HandleTypeDef * hi2c, uint8_t sensor_index)
{
	return movingAvg(MLX90614_ReadTemperature(MLX90614_TOBJ, hi2c), sensor_index, 1);
}*/

float Moving_Avg(float next_num, uint8_t sensor_index)
{
  //Subtract the oldest number from the prev sum, add the new number
  temp_moving_avgs[sensor_index].sum = temp_moving_avgs[sensor_index].sum - temp_moving_avgs[sensor_index].array[temp_moving_avgs[sensor_index].counter] + next_num;
  //Assign the nextNum to the position in the array
  temp_moving_avgs[sensor_index].array[temp_moving_avgs[sensor_index].counter++] = next_num;
  if (temp_moving_avgs[sensor_index].counter >= MOVING_AVG_WINDOW_SIZE)
  {
    temp_moving_avgs[sensor_index].counter = 0;
    temp_moving_avgs[sensor_index].flag = 1;
  }
  //return the average
  if(temp_moving_avgs[sensor_index].flag == 1)
    return temp_moving_avgs[sensor_index].sum / (float)MOVING_AVG_WINDOW_SIZE;
  else
    return 0.0;
}
void Temp_Task(void *pvParameters)
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	for(;;)
    {
        if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
        	for(uint8_t i = 0; i < DAQ_NO_OF_TEMP_SENSORS; i++)
        		MLX90614_Read_Data(i);
            xSemaphoreGive(i2c_mutex);
            for(uint8_t i = 0; i < DAQ_NO_OF_TEMP_SENSORS; i++)
            	wheel_temps[i] = MLX90614_Process_Temp(i);
            daq_can_msg_t can_msg_temp = {};
            daq_can_msg_temp_t encoder_msg_temp = {};
            encoder_msg_temp.temp_front_left = (uint16_t)(wheel_temps[TEMP_FRONT_LEFT] 	* DAQ_ACCURACY_TEMPERATURE);
            encoder_msg_temp.temp_front_right= (uint16_t)(wheel_temps[TEMP_FRONT_RIGHT]	* DAQ_ACCURACY_TEMPERATURE);
            encoder_msg_temp.temp_rear_left 	= (uint16_t)(wheel_temps[TEMP_REAR_LEFT] 	* DAQ_ACCURACY_TEMPERATURE);
            encoder_msg_temp.temp_rear_right = (uint16_t)(wheel_temps[TEMP_REAR_RIGHT] 	* DAQ_ACCURACY_TEMPERATURE);
            can_msg_temp.id = DAQ_CAN_ID_TEMPERATURE;
            can_msg_temp.size = 8;
            can_msg_temp.data = *((uint64_t*)&encoder_msg_temp);
            DAQ_CAN_Msg_Enqueue(&can_msg_temp);
        }
        vTaskDelayUntil(&xLastWakeTime, 13);
    }
}
/*void Send_UART_Ambient_Temperature(UART_HandleTypeDef * huart,I2C_HandleTypeDef * hi2c)
{
    char buffer[50];
    char str[25];
    float ambient_temp = MLX90614_ReadAmbientTemp(hi2c);

    float_to_string(ambient_temp, str,2);
   	sprintf(buffer, "Ambient Temperature: %s°C\n\r",str);
    HAL_UART_Transmit(huart, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void Send_UART_Object_Temperature(UART_HandleTypeDef * huart,I2C_HandleTypeDef * hi2c)
{
    char buffer[50];
    char str[25];
    float Object_temp = MLX90614_ReadObjectTemp(hi2c);
    float_to_string(Object_temp, str,2);
    sprintf(buffer,"Object Temperature: %s°C\n\r",str);
    HAL_UART_Transmit(huart, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}*/


/*void float_to_string(float number, char *str, int precision) {
    int int_part = (int)number;
    float fraction = number - (float)int_part;
    int i = 0;
    // Handle negative numbers
    if (number < 0) {
        *str++ = '-';
        int_part = -int_part;
        fraction = -fraction;
    }

    // Convert integer part
    sprintf(str, "%d", int_part);
    while (*str) str++; // Move to end of current string

    *str++ = '.'; // Decimal point

    // Convert fractional part
    for ( i = 0; i < precision; i++) {
			 int digit;
        fraction *= 10;
        digit = (int)fraction;
        *str++ = '0' + digit;
        fraction -= digit;
    }

    *str = '\0'; // Null terminator
}*/

