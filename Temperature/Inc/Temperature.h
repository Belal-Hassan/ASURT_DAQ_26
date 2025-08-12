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

/**
 * @addtogroup Temp_Module
 * @{
 */

/** @brief Base I2C address for first temperature sensor. */
#define TEMP_I2C_BASE_ADDRESS       DAQ_TEMP_I2C_BASE_ADDRESS

/** @brief Address spacing between consecutive sensors on the I2C bus. */
#define TEMP_I2C_ADDRESS_SPACING    DAQ_TEMP_I2C_ADDRESS_SPACING

/** @brief Number of temperature sensors in the system. */
#define TEMP_NO_OF_SENSORS          DAQ_NO_OF_TEMP_SENSORS

/** @brief Number of samples used in moving average filter. */
#define MOVING_AVG_WINDOW_SIZE      5

/**
 * @brief Wheel position identifiers for temperature sensors.
 */
typedef enum {
    TEMP_FRONT_LEFT,   /**< Front-left wheel sensor.  */
    TEMP_FRONT_RIGHT,  /**< Front-right wheel sensor. */
    TEMP_REAR_LEFT,    /**< Rear-left wheel sensor.   */
    TEMP_REAR_RIGHT,   /**< Rear-right wheel sensor.  */
} temp_wheels_t;

/**
 * @brief Moving average filter state.
 */
typedef struct {
    float array[MOVING_AVG_WINDOW_SIZE]; /**< Sample buffer. */
    float sum;                           /**< Running sum of samples. */
    unsigned char counter;               /**< Index of next sample position. */
    unsigned char flag;                   /**< Set when buffer is filled for the first time. */
} moving_avg_t;

/**
 * @brief Single sensor reading data.
 */
typedef struct {
    uint8_t data[3];     /**< Raw I2C bytes from the sensor. */
    uint16_t raw;        /**< Combined raw value from sensor. */
    float temperature;   /**< Converted temperature in Celsius. */
} temp_sensor_data_t;

/**
 * @brief Stores current and previous readings for change detection.
 */
typedef struct {
    float prev[TEMP_NO_OF_SENSORS];    /**< Previous readings. */
    float current[TEMP_NO_OF_SENSORS]; /**< Current readings.  */
} temp_reading_buffer_t;

/**
 * @brief Applies a moving average filter to temperature readings.
 * @param next_num New temperature reading.
 * @param sensor_index Index of the sensor [0 ... @ref TEMP_NO_OF_SENSORS -1].
 * @return Filtered temperature value, or 0.0 if buffer is not yet filled.
 */
float Moving_Avg(float next_num, uint8_t sensor_index);

/**
 * @brief Reads raw temperature data from a specific sensor.
 * @param sensor_index Index of the sensor [0 .. TEMP_NO_OF_SENSORS-1].
 */
void Temp_ReadRaw(uint8_t sensor_index);

/**
 * @brief Converts raw sensor data to Celsius and applies moving average filter.
 * @param sensor_index Index of the sensor [0 .. TEMP_NO_OF_SENSORS-1].
 * @return Filtered temperature in Celsius.
 */
float Temp_Process(uint8_t sensor_index);

/**
 * @brief Initializes the temperature module with the provided I2C handle.
 * @param hi2c Pointer to HAL I2C handle.
 */
void Temp_Init(I2C_HandleTypeDef * hi2c);

/**
 * @brief FreeRTOS task that periodically reads and transmits temperatures over CAN.
 * @param pvParameters Not used.
 */
void Temp_Task(void *pvParameters);
/** @}*/

#endif /* TEMPERATURE_INC_TEMPERATURE_H_ */
