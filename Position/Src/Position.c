/*
 * Position.c
 *
 *  Created on: Mar 4, 2025
 *      Author: Belal
 */
#include "Position.h"

adc_median_filter_t adc_filters[DAQ_NO_OF_ADC_SENSORS];
volatile uint16_t adc_raw_values[DAQ_NO_OF_ADC_SENSORS] = {0,0,0,0,0,0};
adc_readings_t adc_readings;


void Median_Init(median_filter_t* this, uint16_t* buffer, uint16_t** pt_buffer_sorted, uint16_t size)
{
    if (this == NULL)
    {
        return;
    }
    else if ((buffer == NULL)||(pt_buffer_sorted == NULL)||(size < 3)||(size > UINT8_MAX))
    {
        this->init=false;
        return;
    }

    this->buffer=buffer;
    this->pt_buffer_sorted=pt_buffer_sorted;
    this->size=size;
    this->index=0;
    this->initial_knuth_gap=1;
    this->init=true;

    Median_Buffer_Clear(this);
    Median_Buffer_Init(this);
    Median_SortingGap_Init(this);
}

uint16_t Median_Filter(median_filter_t* this, uint16_t input)
{
    if ((this == NULL)||(this->init==false))
        return 0;

    this->buffer[this->index] = input;

    /*increment index*/
    this->index = (this->index + 1) % this->size;

    Median_Buffer_ShellSort(this);

    return Median_ValueGet(this);
}

uint8_t Median_IterationGet(median_filter_t* this)
{
    if ((this == NULL)||(this->init==false))
        return 0;

    return this->iteration_count;
}

void Median_SortingGap_Init(median_filter_t* this)
{
    /* calculate initial gap using Knuth sequence for Shell-Sort */
    this->initial_knuth_gap=1;
    while (this->initial_knuth_gap < this->size / 3)
    {
        this->initial_knuth_gap = this->initial_knuth_gap * 3 + 1;
    }
}

uint16_t Median_ValueGet(median_filter_t* this)
{
    if (this->size % 2)
    {   /* return the middle value */
        return *this->pt_buffer_sorted[(this->size / 2)];
    }
    else
    {   /* return the average of the two middle values */
        return (*this->pt_buffer_sorted[(this->size / 2)] + *this->pt_buffer_sorted[(this->size / 2) - 1]) / 2;
    }
}

void Median_Buffer_Clear(median_filter_t* this)
{
    if (this == NULL)
        return;

    for (uint8_t i = 0; i < this->size; i++)
        this->buffer[i] = 0;
}

void Median_Buffer_Init(median_filter_t* this)
{
    for (uint8_t i = 0; i < this->size; i++)
        this->pt_buffer_sorted[i] = &this->buffer[i];
}

void Median_Buffer_ShellSort(median_filter_t* this)
{
    if (this == NULL)
        return;

    this->iteration_count = 0;

    uint8_t gap=this->initial_knuth_gap;

    while (gap > 0)
    {
        for (uint8_t  index = gap; index < this->size; index++)
        {
            uint16_t* currentElement = this->pt_buffer_sorted[index];

            /* If left element is larger, swap until correct position is found. */
            while (index >=gap && *this->pt_buffer_sorted[index - gap] > *currentElement)
            {
                Swap_PtPt(&this->pt_buffer_sorted[index - gap],&this->pt_buffer_sorted[index]);
                index-= gap;
                this->iteration_count++;
            }
            this->pt_buffer_sorted[index] = currentElement;
        }

        /* calculate next gap, using Knuth sequence */
        if (gap!=1)
        {
            gap = (gap - 1) / 3;
        }else
        {
            gap=0;
        }
    }
}

void Swap_PtPt(uint16_t **a, uint16_t **b)
{
    uint16_t *temp = *a;
    *a = *b;
    *b = temp;
}

void ADC_Sensors_Init(ADC_HandleTypeDef *hadc)
{
	HAL_ADC_Start_DMA(hadc, adc_raw_values, DAQ_NO_OF_ADC_SENSORS);
	for(uint8_t i = 0; i < DAQ_NO_OF_ADC_SENSORS; i++)
		Median_Init(&adc_filters[i].filter, adc_filters[i].buffer, adc_filters[i].pt_buffer_sorted, MEDIAN_FILTER_BUFFER_SIZE);
}

void ADC_Sensors_Process(volatile uint16_t *raw_adc_values)
{
	for(uint8_t i = 0; i < DAQ_NO_OF_ADC_SENSORS; i++)
		adc_filters[i].filtered_value =  Median_Filter(&adc_filters[i].filter, raw_adc_values[i]);
	for(uint8_t i = 0; i < DAQ_NO_OF_ADC_SENSORS; i++)
		adc_filters[i].percent = (float)adc_filters[i].filtered_value / 4096.0;

}
void ADC_Task(void *pvParameters)
{
	TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
	for( ;; )
    {
		ADC_Sensors_Process(adc_raw_values);
		for(uint8_t i = 0; i < DAQ_NO_OF_ADC_SENSORS; i++)
			adc_readings.current[i] = adc_filters[i].filtered_value;
		if(DAQ_CheckChange(adc_readings.current[SUSPENSION_FRONT_LEFT], adc_readings.prev[SUSPENSION_FRONT_LEFT], 	DAQ_MIN_CHANGE_SUSPENSION) ||
		   DAQ_CheckChange(adc_readings.current[SUSPENSION_FRONT_RIGHT],adc_readings.prev[SUSPENSION_FRONT_RIGHT], 	DAQ_MIN_CHANGE_SUSPENSION) ||
		   DAQ_CheckChange(adc_readings.current[SUSPENSION_REAR_LEFT], 	adc_readings.prev[SUSPENSION_REAR_LEFT],	DAQ_MIN_CHANGE_SUSPENSION) ||
		   DAQ_CheckChange(adc_readings.current[SUSPENSION_REAR_RIGHT], adc_readings.prev[SUSPENSION_REAR_RIGHT],	DAQ_MIN_CHANGE_SUSPENSION) ||
		   DAQ_CheckChange(adc_readings.current[PRESSURE_REAR_LEFT], 	adc_readings.prev[PRESSURE_REAR_LEFT], 		DAQ_MIN_CHANGE_PRESSURE)   ||
		   DAQ_CheckChange(adc_readings.current[PRESSURE_REAR_RIGHT], 	adc_readings.prev[PRESSURE_REAR_RIGHT], 	DAQ_MIN_CHANGE_PRESSURE))
		{
			daq_can_msg_t can_msg_adc = {};
			daq_can_msg_adc_t encoder_msg_adc = {};
			encoder_msg_adc.suspension_front_left 	= (uint64_t)(adc_filters[SUSPENSION_FRONT_LEFT].percent * DAQ_ACCURACY_SUSPENSION);
			encoder_msg_adc.suspension_front_right	= (uint64_t)(adc_filters[SUSPENSION_FRONT_RIGHT].percent* DAQ_ACCURACY_SUSPENSION);
			encoder_msg_adc.suspension_rear_left 	= (uint64_t)(adc_filters[SUSPENSION_REAR_LEFT].percent	* DAQ_ACCURACY_SUSPENSION);
			encoder_msg_adc.suspension_rear_right 	= (uint64_t)(adc_filters[SUSPENSION_REAR_RIGHT].percent * DAQ_ACCURACY_SUSPENSION);
			encoder_msg_adc.pressure_rear_left 		= (uint64_t)(adc_filters[PRESSURE_REAR_LEFT].percent	* DAQ_ACCURACY_PRESSURE);
			encoder_msg_adc.pressure_rear_right 	= (uint64_t)(adc_filters[PRESSURE_REAR_RIGHT].percent	* DAQ_ACCURACY_PRESSURE);
			can_msg_adc.id = DAQ_CAN_ID_ADC;
			can_msg_adc.data = *((uint64_t*)(&encoder_msg_adc));
			can_msg_adc.size = 8;
			DAQ_CAN_Msg_Enqueue(&can_msg_adc);
			for(uint8_t i = 0; i < DAQ_NO_OF_ADC_SENSORS; i++)
				adc_readings.prev[i] = adc_readings.current[i];
		}
		vTaskDelayUntil(&xLastWakeTime, 7);
	}
}
