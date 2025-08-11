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
/**
 * @addtogroup Prox_Module
 * @{
 */

/**
 * @brief Calculates vehicle speed (km/h) from two wheel RPM values.
 *
 * Uses the average of two wheel RPMs, multiplies by the tire circumference, and converts
 * from revolutions per minute to kilometers per hour.
 *
 * @param rpm1 RPM of the first wheel.
 * @param rpm2 RPM of the second wheel.
 * @return Calculated speed in km/h.
 */
#define PROX_CALCULATE_SPEED(rpm1, rpm2) ((((rpm1 + rpm2) / 2.0) * DAQ_TIRE_CIRCUMFERENCE * 60.0) / 1000.0)

/** Number of wheels monitored by the proximity sensor system. */
#define PROX_NO_OF_WHEELS 			4

/** Size of DMA buffer for each wheel's input capture readings. */
#define PROX_DMA_WHEEL_BUFFER_SIZE	10

/**
 * @brief Identifiers for wheel DMA buffers.
 */
typedef enum {
    FRONT_LEFT_BUFF,
    FRONT_RIGHT_BUFF,
    REAR_LEFT_BUFF,
    REAR_RIGHT_BUFF
}wheel_buffer_t;
/**
 * @brief Structure holding previous and current RPM readings for all wheels.
 */
typedef struct{
	double prev[PROX_NO_OF_WHEELS];
	double current[PROX_NO_OF_WHEELS];
}prox_rpm_buffer_t;
/**
 * @brief Initialize the proximity sensor system.
 *
 * Starts timer input capture with DMA for all wheels.
 *
 * @param htim Pointer to timer handle used for input capture.
 * @param hdma Array of pointers to DMA handles for each wheel.
 */
void Prox_Init(TIM_HandleTypeDef *htim, DMA_HandleTypeDef* hdma[4]);
/**
 * @brief Periodic FreeRTOS task to read DMA buffers, calculate RPM and speed,
 *        and publish CAN messages.
 *
 * Handles low-speed detection by counting slow readings, resets DMA buffers,
 * and sends CAN messages only when meaningful RPM changes are detected.
 *
 * @param pvParameters Unused task parameters.
 */
void Prox_Task(void *pvParameters);
/** @}*/

#endif /* PROXIMITY_INC_PROXIMITY_H_ */
