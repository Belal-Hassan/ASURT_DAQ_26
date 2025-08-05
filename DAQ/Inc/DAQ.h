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
	PROX_TASK = 0,
	IMU_TASK,
	GPS_TASK,
	ADC_TASK,
	TEMP_TASK,
	CAN_TASK,
	WWDG_TASK
}daq_task_handle_t; // arranged according to priority, except for the wwdg task.
typedef enum{
	DAQ_BKPSRAM_UNINITIALIZED = 6,
	DAQ_BKPSRAM_INITIALIZED,
}daq_bkpsram_state_t;
typedef enum{
	DAQ_FAULT_LOGGED = 9,
	DAQ_FAULT_READ
}daq_log_status_t;
typedef struct{
	daq_bkpsram_state_t bkpsram_state;
	daq_log_status_t	log_status;
}daq_status_word_t;
typedef enum{
	DAQ_RESET_REASON_NONE = 0,
	DAQ_RESET_REASON_HARDFAULT = 7,
	DAQ_RESET_REASON_MEMMANAGE,
	DAQ_RESET_REASON_BUSFAULT,
	DAQ_RESET_REASON_USAGEFAULT,
	DAQ_RESET_REASON_ERRORHANDLER
}daq_reset_reason_t;
typedef enum{
	DAQ_READ_PREVIOUS_LOG,
	DAQ_READ_CURRENT_LOG,
	DAQ_READ_STATUS_WORDS
}daq_bkpsram_read_type_t;
typedef enum{
	DAQ_WRITE_LOG,
	DAQ_WRITE_BKPSRAM_STATE,
	DAQ_WRITE_LOG_STATUS,
	DAQ_CLEAR_CURRENT_LOG
}daq_bkpsram_write_type_t;
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
	uint8_t  seconds;
	uint8_t  minutes;
	uint8_t  hours;
	uint16_t counter;
}daq_timestamp_t;

typedef struct __attribute__((packed)){
	TickType_t start_tick: 24;
	uint32_t entry_count: 4;
	uint32_t error_count: 3;
	uint32_t task_kicked: 1;
	TickType_t runtime	: 24;
}task_stats_t;

typedef struct{
	task_stats_t tasks[DAQ_NO_OF_READ_TASKS];
}daq_fault_record_t;

typedef struct __attribute__((packed)) {
    uint8_t reset_reason;
    uint8_t fault_status;
    uint32_t fault_address;
    //uint32_t stack_frame[8]; // suggested future improvement.
    daq_fault_record_t task_records;
    daq_timestamp_t timestamp;
}fault_log_t;

typedef struct{
    fault_log_t prev;
    fault_log_t current;
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
void DAQ_FaultLog_Init(void);
/**
 * @brief Dequeues one COMM_can_message_t from the FreeRTOS CAN queue.
 *
 * @return COMM_can_message_t: the dequeued message.
 * @attention This function should only be used in CAN task. If the queue is empty, it will block the task until the queue is not empty.
 */
BaseType_t DAQ_CAN_Msg_Dequeue(daq_can_msg_t* msg);
daq_fault_log_t DAQ_FaultLog_Read(void);
void DAQ_FaultLog_Write(fault_log_t log);
void DAQ_CAN_Task(void *pvParameters);

#endif /* DAQ_DAQ_H_ */
