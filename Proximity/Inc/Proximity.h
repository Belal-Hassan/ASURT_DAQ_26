/*
 * Proximity.h
 *
 *  Created on: Apr 15, 2025
 *      Author: Belal
 */

#ifndef PROXIMITY_INC_PROXIMITY_H_
#define PROXIMITY_INC_PROXIMITY_H_


#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"
//#include "FreeRTOS.h"
#include "DAQ.h"

#define PROX_CALCULATE_SPEED(rpm1, rpm2) ((((rpm1 + rpm2) / 2.0) * DAQ_TIRE_CIRCUMFERENCE * 60.0) / 1000.0)
#define PROX_NO_OF_WHEELS 			4
#define PROX_DMA_WHEEL_BUFFER_SIZE	16

typedef enum {
    FRONT_LEFT_BUFF,
    FRONT_RIGHT_BUFF,
    REAR_LEFT_BUFF,
    REAR_RIGHT_BUFF
}wheel_buffer_t;

typedef struct{
	double prev[PROX_NO_OF_WHEELS];
	double current[PROX_NO_OF_WHEELS];
}prox_rpm_buffer_t;

void Prox_Init(TIM_HandleTypeDef *htim, DMA_HandleTypeDef* hdma[4]);
void Prox_Task(void *pvParameters);

#endif /* PROXIMITY_INC_PROXIMITY_H_ */
