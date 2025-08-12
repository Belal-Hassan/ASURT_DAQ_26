/*
 * Position.h
 *
 *  Created on: Mar 4, 2025
 *      Author: Belal
 */

#ifndef POSITION_POSITION_H_
#define POSITION_POSITION_H_

#include "main.h"
#include "DAQ.h"

/**
 * @addtogroup ADC_Module
 * @{
 */

/**
 * @def MEDIAN_FILTER_BUFFER_SIZE
 * @brief Size of the median filter buffer.
 *
 * This constant defines the number of samples used in the median filter window.
 *
 * @note An odd-sized buffer ensures a single middle value for median calculation without
 * requiring averaging of two central values.
 */
#define MEDIAN_FILTER_BUFFER_SIZE  	5

/**
 * @enum adc_sensors_t
 * @brief Enum of all ADC sensors used.
 *
 */
typedef enum{
	SUSPENSION_FRONT_LEFT,    ///< Front-left suspension height sensor
	SUSPENSION_FRONT_RIGHT,   ///< Front-right suspension height sensor
	SUSPENSION_REAR_LEFT,     ///< Rear-left suspension height sensor
	SUSPENSION_REAR_RIGHT,    ///< Rear-right suspension height sensor
	PRESSURE_REAR_LEFT,       ///< Rear-left brake pressure sensor
	PRESSURE_REAR_RIGHT       ///< Rear-right brake pressure sensor
}adc_sensors_t;

/**
 * @struct median_filter_t
 * @brief State struct for the median filter algorithm.
 *
 * Maintains the internal state required for the running median calculation. Uses
 * pointer-based sorting to minimize data movement during buffer updates. The
 * Shell-sort implementation provides efficient sorting for small buffer sizes.
 *
 */
typedef struct{
	uint16_t* buffer;           ///< Direct sample buffer (raw ADC values)
	uint16_t** pt_buffer_sorted;///< Pointer array to buffer elements (for sorting)
    uint8_t size;               ///< Filter buffer size (must match MEDIAN_FILTER_BUFFER_SIZE)
    uint8_t index;              ///< Current write position in circular buffer
    uint8_t iteration_count;    ///< Sort iteration counter (for performance monitoring)
    uint8_t initial_knuth_gap;  ///< Initial gap value for Shell-sort (Knuth sequence)
    bool init;                  ///< Initialization flag (true = ready for use)
}median_filter_t;

/**
 * @struct adc_median_filter_t
 * @brief Complete filter context for a single ADC sensor channel.
 *
 * Combines the median filter state with additional channel-specific data
 * including filtered output values and percentage conversions.
 */
typedef struct{
	uint16_t buffer[MEDIAN_FILTER_BUFFER_SIZE];    ///< Sample storage buffer
	uint16_t* pt_buffer_sorted[MEDIAN_FILTER_BUFFER_SIZE]; ///< Sorted pointer array
	uint16_t filtered_value;     ///< Current median-filtered ADC value (0-4095)
	float percent;               ///< Normalized value (0.0-1.0 of full scale)
	median_filter_t filter;      ///< Underlying filter state machine
}adc_median_filter_t;

/**
 * @struct adc_reading_buffer_t
 * @brief Buffer for tracking previous and current sensor states.
 *
 * Maintains both previous and current filtered values to calculate changes
 * in sensor readings. Used to trigger CAN message transmission only
 * when significant changes occur.
 */
typedef struct{
	uint16_t prev[DAQ_NO_OF_ADC_SENSORS];  ///< Previous filtered values
	uint16_t current[DAQ_NO_OF_ADC_SENSORS];///< Current filtered values
}adc_reading_buffer_t;

/**
 * @brief Initialize a median filter instance.
 *
 * Configures the filter with buffer pointers and initializes internal state.
 * Validates input parameters and sets up the sorting mechanism. Should be
 * called once per filter instance before processing samples.
 *
 * @param this Pointer to filter instance to initialize
 * @param buffer Raw sample buffer (must be size elements)
 * @param pt_buffer_sorted Pointer array for sorted references (must be size elements)
 * @param size Filter window size (must be 3-255 and odd)
 */
void Median_Init(median_filter_t* this, uint16_t* buffer, uint16_t** pt_buffer_sorted, uint16_t size);

/**
 * @brief Process a new sample through the median filter.
 *
 * Adds a new sample to the filter window, updates the sorted pointer array,
 * and returns the current median value. Handles circular buffer management
 * and automatic sorting.
 *
 * @param this Pointer to initialized filter instance
 * @param input New ADC sample value to process
 * @return Current median value of the filter window
 */
uint16_t Median_Filter(median_filter_t* this, uint16_t input);

/**
 * @brief Get the last sort iteration count.
 *
 * Returns the number of iterations performed during the most recent sorting
 * operation. Useful for performance monitoring and ensuring the filter meets
 * real-time constraints.
 *
 * @param this Pointer to filter instance
 * @return Number of sorting iterations (0 if uninitialized)
 */
uint8_t Median_IterationGet(median_filter_t* this);

/**
 * @brief Clear all filter buffers.
 *
 * Resets all sample buffers to zero values. Does not reinitialize pointers
 * or sorting state - primarily used for debugging or error recovery.
 *
 * @param this Pointer to filter instance
 */
void Median_Buffer_Clear(median_filter_t* this);

/**
 * @brief Sort the filter buffer using Shell-sort.
 *
 * Re-sorts the pointer array after new sample insertion. Uses Knuth's gap
 * sequence for optimal performance on small datasets. Updates iteration count.
 *
 * @param this Pointer to filter instance
 */
void Median_Buffer_ShellSort(median_filter_t* this);

/**
 * @brief Initialize buffer pointers.
 *
 * Sets up the pointer array to reference the raw sample buffer elements.
 * Must be called after buffer allocation but before first sample processing.
 *
 * @param this Pointer to filter instance
 */
void Median_Buffer_Init(median_filter_t* this);

/**
 * @brief Get the current median value.
 *
 * Calculates and returns the median value from the sorted pointer array.
 * Handles both odd and even-sized buffers appropriately.
 *
 * @param this Pointer to filter instance
 * @return Median value of current window
 */
uint16_t Median_ValueGet(median_filter_t* this);

/**
 * @brief Initialize Knuth gap sequence.
 *
 * Calculates the initial gap value for Shell-sort based on Knuth's sequence:
 * gap = 3*gap + 1 until gap >= size/3. Optimizes sort performance.
 *
 * @param this Pointer to filter instance
 */
void Median_SortingGap_Init(median_filter_t* this);

/**
 * @brief Initialize all ADC sensor channels.
 *
 * Configures DMA for ADC conversions and initializes median filters for
 * all sensor channels. Should be called during system startup.
 *
 * @param hadc Pointer to ADC hardware handle
 */
void ADC_Sensors_Init(ADC_HandleTypeDef *hadc);

/**
 * @brief Process raw ADC values through filters.
 *
 * Applies median filtering to all raw ADC channels and converts results
 * to normalized percentage values (0.0-1.0). Updates internal filter states.
 *
 * @param raw_adc_values Array of raw ADC values from DMA buffer
 */
void ADC_Sensors_Process(volatile uint16_t *raw_adc_values);

/**
 * @brief Main task for ADC processing and CAN transmission.
 *
 * Periodically processes ADC data, detects significant changes, and
 * transmits updated values via CAN when thresholds are exceeded.
 * Implements rate control through vTaskDelayUntil.
 *
 * @param pvParameters Unused task parameter (required by FreeRTOS)
 */
void ADC_Task(void *pvParameters);

/** @}*/


#endif /* POSITION_POSITION_H_ */
