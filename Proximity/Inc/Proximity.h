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
#include "FreeRTOS.h"
#include "Proximity_Config.h"
#include "DAQ.h"

#define PROX_TIMER_FREQ ( ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) ? \
                         ((float)HAL_RCC_GetPCLK1Freq() * 2.0f) : \
                         ((float)HAL_RCC_GetPCLK1Freq()) ) / ((float)TIM3->PSC + 1.0f)

static inline float Get_TIM3_FrequencyHz(void) {
    float pclk = (RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1 ?
                  (float)HAL_RCC_GetPCLK1Freq() * 2.0f :
                  (float)HAL_RCC_GetPCLK1Freq();
    return pclk / ((float)TIM3->PSC + 1.0f);
}

#define PROX_CALCULATE_SPEED(rpm1, rpm2) ((((rpm1 + rpm2) / 2.0) * DAQ_TIRE_CIRCUMFERENCE * 60) / 1000)


void Prox_Init(TIM_HandleTypeDef *htim, DMA_HandleTypeDef* hdma[4]);
void Prox_Task(void *pvParameters);

typedef enum {
    FRONT_LEFT_BUFF,
    FRONT_RIGHT_BUFF,
    REAR_LEFT_BUFF,
    REAR_RIGHT_BUFF
} Wheel_buffer_t;

#endif /* PROXIMITY_INC_PROXIMITY_H_ */
