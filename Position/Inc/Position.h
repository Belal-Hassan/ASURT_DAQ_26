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
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"

#define MEDIAN_FILTER_BUFFER_SIZE  	5

typedef enum{
	SUSPENSION_FRONT_LEFT,
	SUSPENSION_FRONT_RIGHT,
	SUSPENSION_REAR_LEFT,
	SUSPENSION_REAR_RIGHT,
	PRESSURE_REAR_LEFT,
	PRESSURE_REAR_RIGHT
}adc_sensors_t;

typedef struct{
	uint16_t* buffer;
	uint16_t** pt_buffer_sorted;
    uint8_t size;
    uint8_t index;
    uint8_t iteration_count;
    uint8_t initial_knuth_gap;
    bool init;
}median_filter_t;

typedef struct{
	uint16_t buffer[MEDIAN_FILTER_BUFFER_SIZE];
	uint16_t* pt_buffer_sorted[MEDIAN_FILTER_BUFFER_SIZE];
	uint16_t filtered_value;
	float percent;
	median_filter_t filter;
}adc_median_filter_t;

typedef struct{
	uint16_t prev[DAQ_NO_OF_ADC_SENSORS];
	uint16_t current[DAQ_NO_OF_ADC_SENSORS];
}adc_readings_t;

void Median_Init(median_filter_t* this, uint16_t* buffer, uint16_t** pt_buffer_sorted, uint16_t size);
uint16_t Median_Filter(median_filter_t* this, uint16_t input);
uint8_t Median_IterationGet(median_filter_t* this);
void Median_Buffer_Clear(median_filter_t* this);
void Median_Buffer_ShellSort(median_filter_t* this);
void Median_Buffer_Init(median_filter_t* this);
uint16_t Median_ValueGet(median_filter_t* this);
void Swap_PtPt(uint16_t **a, uint16_t **b);
void Median_SortingGap_Init(median_filter_t* this);
void ADC_Sensors_Init(ADC_HandleTypeDef *hadc);
void ADC_Sensors_Process(volatile uint16_t *raw_adc_values);
void ADC_Task(void *pvParameters);


#endif /* POSITION_POSITION_H_ */
