/*
 * Temperature.h
 *
 *  Created on: Jul 1, 2025
 *      Author: Belal
 */

#ifndef TEMPERATURE_INC_TEMPERATURE_H_
#define TEMPERATURE_INC_TEMPERATURE_H_

#include "main.h"
#include "DAQ.h"
#include "Temperature_Config.h"

typedef enum{
	TEMP_FRONT_LEFT,
	TEMP_FRONT_RIGHT,
	TEMP_REAR_LEFT,
	TEMP_REAR_RIGHT,
}temp_wheels_t;

typedef struct{
    float array[MOVING_AVG_WINDOW_SIZE];
    float sum;
    unsigned char counter;
    unsigned char flag;
} moving_avg_t;

typedef struct{
	uint8_t data[3];
	uint16_t raw;
	float temperature;
}temp_sensor_data_t;


void MLX90614_Read_Data(uint8_t sensor_index);
float MLX90614_Process_Temp(uint8_t sensor_index);
float Moving_Avg(float next_num, uint8_t sensor_index);
void Temp_Init(I2C_HandleTypeDef * hi2c);
void Temp_Task(void *pvParameters);
void float_to_string(float number, char *str, int precision);


#endif /* TEMPERATURE_INC_TEMPERATURE_H_ */
