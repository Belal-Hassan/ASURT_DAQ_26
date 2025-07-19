/*
 * IMU.h
 *
 *  Created on: Jul 17, 2025
 *      Author: Belal
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "Adafruit_BNO055.h"
#include "DAQ.h"

typedef struct{
	double x;
	double y;
	double z;
}imu_vector_t;

typedef struct{
	imu_vector_t prev;
	imu_vector_t current;
}imu_readings_t;

void IMU_Init(I2C_HandleTypeDef* hi2c, adafruit_bno055_opmode_t mode, adafruit_bno055_axis_map_t map);
void IMU_Task(void *pvParameters);

#endif /* INC_IMU_H_ */
