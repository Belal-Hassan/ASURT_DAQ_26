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

#define TEMP_I2C_BASE_ADDRESS		DAQ_TEMP_I2C_BASE_ADDRESS
#define TEMP_I2C_ADDRESS_SPACING 	DAQ_TEMP_I2C_ADDRESS_SPACING
#define TEMP_NO_OF_SENSORS			DAQ_NO_OF_TEMP_SENSORS
// Moving average parameters
#define MOVING_AVG_WINDOW_SIZE 		5

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

typedef struct{
	float prev[TEMP_NO_OF_SENSORS];
	float current[TEMP_NO_OF_SENSORS];
}temp_reading_buffer_t;


float Moving_Avg(float next_num, uint8_t sensor_index);
void Temp_ReadRaw(uint8_t sensor_index);
float Temp_Process(uint8_t sensor_index);
void Temp_Init(I2C_HandleTypeDef * hi2c);
void Temp_Task(void *pvParameters);


#endif /* TEMPERATURE_INC_TEMPERATURE_H_ */
