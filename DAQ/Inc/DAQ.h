/*
 * DAQ.h
 *
 *  Created on: Mar 4, 2025
 *      Author: Belal
 */

#ifndef DAQ_DAQ_H_
#define DAQ_DAQ_H_

#include <stdint.h>
#include "main.h"
#include "DAQ_Config.h"
/* =========================================== FREERTOS INCLUDES =========================================== */
#include "FreeRTOS.h"
#include "queue.h"
/* =========================================== STM32HAL INCLUDES =========================================== */
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"

#define DAQ_CheckChange(current_data, prev_data, min_change) (fabs(current_data - prev_data) >= min_change)

/* CAN */

typedef enum{
	I2C_DMA_NO_DEVICE = 0,
	I2C_DMA_GPS
}daq_i2c_dma_devices_t;

typedef enum {
	ADC_TASK = 0,
	PROX_TASK,
	IMU_TASK,
	GPS_TASK,
	TEMP_TASK,
	CAN_TASK
}daq_task_handles_t;
/**
 * @brief Message to be enqueued in the CAN queue.
 *
 */
typedef struct{
    uint64_t data; /*!< Data to be sent. Maximum of 8 bytes (as specified by CAN protocol). */
    uint16_t id;   /*!< 11-bit CAN message id. Must be the least-significant 11 bits. */
    uint8_t size;  /*!< Size of the data in bytes. */
} daq_can_msg_t;
/**
 * @brief Format for messages from IMU to be included in a COMM_can_message_t object.
 * Can include accleration or angles.
 *
 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} daq_can_msg_imu_t;
/**
 * @brief Format for messages from ADC_DEV to be included in a COMM_can_message_t object.
 * Includes travel sensor and pressure sensor values.
 *
 */
typedef struct
{
    uint64_t suspension_front_left 	: 10;
    uint64_t suspension_front_right : 10;
    uint64_t suspension_rear_left 	: 10;
    uint64_t suspension_rear_right 	: 10;

    uint64_t pressure_rear_left 	: 10;
    uint64_t pressure_rear_right 	: 10;
} daq_can_msg_adc_t;
/**
 * @brief Format for messages from proximity to be included in a COMM_can_message_t object.
 * Contains RPM for the 4 wheels and the angle of the steering wheel from rotary encoder,
 * and the speed of the car in km/h
 */
typedef struct
{
    uint64_t rpm_front_left : 11;
    uint64_t rpm_front_right: 11;
    uint64_t rpm_rear_left 	: 11;
    uint64_t rpm_rear_right : 11;

    uint64_t encoder_angle 	: 10;
    uint64_t Speedkmh 		: 8;
} daq_can_msg_prox_t;
/**
 * @brief Format for the message from GPS to be included in a COMM_can_message_t object.
 * Contains latitude and longitude as floats in decimal format.
 */
typedef struct
{
    float latitude;
    float longitude;
} daq_can_msg_gps_t;
/**
 * @brief Format for the message from temperature sensors to be included in a
 * COMM_can_message_t object.
 *
 * Contains each wheel's temperature sent as a uint16_t value. The original
 * float temperature (in degrees Celsius) is multiplied by 100 and cast to
 * uint16_t to preserve two decimal places of accuracy during transmission.
 */
typedef struct
{
    uint16_t temp_front_left;
    uint16_t temp_front_right;
    uint16_t temp_rear_left;
    uint16_t temp_rear_right;
} daq_can_msg_temp_t;
/**
 * @brief IDs for CAN messages.
 *
 */
typedef enum
{
	DAQ_CAN_ID_IMU_ANGLE = DAQ_CAN_BASE_ID,
	DAQ_CAN_ID_IMU_ACCEL,
	DAQ_CAN_ID_ADC,
	DAQ_CAN_ID_PROX_ENCODER,
	DAQ_CAN_ID_GPS,
	DAQ_CAN_ID_TEMP,
} daq_can_id_t;

typedef struct{
	uint8_t sec;
	uint8_t min;
	uint8_t hrs;
	uint8_t CanMsgLongLat[8];
	uint8_t CanMsgTimeSpeed[8];
	double speed;
	double longitude;
	double latitude;
}daq_gps_data_t;

typedef struct{
	uint8_t  seconds;
	uint8_t  minutes;
	uint8_t  hours;
	uint16_t counter;
}daq_timestamp_t;

typedef struct {
    uint32_t reset_reason;
    uint32_t fault_type;
    uint32_t fault_address;
    uint32_t stacked_pc;
    daq_timestamp_t timestamp;
    uint8_t  valid;
}daq_fault_log_t;
/**
 * @brief Adds one CAN message to the FreeRTOS queue, to be transmitted on CAN bus by the CAN task.
 * @param message Pointer to CAN message object to be enqueued.
 */
void DAQ_CAN_Msg_Enqueue(daq_can_msg_t* msg);

/**
 * @brief Initializes the COMM task by starting CAN communication and setting the can_tx_header.
 *
 * @param can_handle Pointer to CAN handle object.
 * @param can_tx_header Pointer to CAN TX header object.
 */
void DAQ_CAN_Init(CAN_HandleTypeDef *can_handle, CAN_TxHeaderTypeDef* can_tx_header);

/**
 * @brief Dequeues one COMM_can_message_t from the FreeRTOS CAN queue.
 *
 * @return COMM_can_message_t: the dequeued message.
 * @attention This function should only be used in CAN task. If the queue is empty, it will block the task until the queue is not empty.
 */
daq_can_msg_t DAQ_CAN_Msg_Dequeue(void);
void DAQ_CAN_Task(void *pvParameters);

#endif /* DAQ_DAQ_H_ */
