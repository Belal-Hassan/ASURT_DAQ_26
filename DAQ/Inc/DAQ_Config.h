/*
 * DAQ_Config.h
 *
 *  Created on: Mar 4, 2025
 *      Author: Belal
 */

#ifndef DAQ_DAQ_CONFIG_H_
#define DAQ_DAQ_CONFIG_H_

#include <stdint.h>

#define DAQ_TIRE_CIRCUMFERENCE 3.1415 * 18 * 0.0254

#define DAQ_CAN_BASE_ID 				0x71
#define DAQ_NO_OF_I2C_DMA_DEVICES 		1 + 1 // "+ 1" because the first element is unused
#define DAQ_NO_OF_TEMP_SENSORS			1
#define DAQ_GPS_I2C_ADDRESS				0x42 << 1
#define DAQ_IMU_I2C_ADDRESS				0x28 << 1 	//IMU's ADR pin is grounded.
#define DAQ_TEMP_I2C_BASE_ADDRESS		0x5A 		// The "<< 1" is in the Temperature.c file
#define DAQ_TEMP_I2C_ADDRESS_SPACING 	1     	// Subjected to change
#define DAQ_IMU_RST_GPIO_PORT			GPIOC
#define DAQ_IMU_RST_PIN					GPIO_PIN_13

#define DAQ_NO_OF_READ_TASKS			5
#define DAQ_NO_OF_TASKS 				DAQ_NO_OF_READ_TASKS + 2 // added 2 for the wwdg and can tasks.

#define DAQ_NO_OF_SUSPENSION			4
#define DAQ_NO_OF_PRESSURE				2
#define DAQ_NO_OF_ADC_SENSORS  			DAQ_NO_OF_SUSPENSION + DAQ_NO_OF_PRESSURE

#define DAQ_MIN_CHANGE_IMU_ANGLE_X 		0.01
#define DAQ_MIN_CHANGE_IMU_ANGLE_Y 		0.01
#define DAQ_MIN_CHANGE_IMU_ANGLE_Z 		0.1
#define DAQ_MIN_CHANGE_IMU_ACCEL 		0.1
#define DAQ_MIN_CHANGE_PROX_RPM 		1
#define DAQ_MIN_CHANGE_TEMP 			0.1
#define DAQ_MIN_CHANGE_SUSPENSION 		5
#define DAQ_MIN_CHANGE_PRESSURE 		5


/* Accuracies of  */
#define DAQ_ACCURACY_IMU_ANGLE_X        100
#define DAQ_ACCURACY_IMU_ANGLE_Y        100
#define DAQ_ACCURACY_IMU_ANGLE_Z        10
#define DAQ_ACCURACY_IMU_ACCEL   		10
#define DAQ_ACCURACY_SUSPENSION         1000
#define DAQ_ACCURACY_PRESSURE			1000
#define DAQ_ACCURACY_TEMP 				10

#define DAQ_TRACE_RECORDE // Comment if tracing is not needed

#define DAQ_BKPSRAM_BASE_ADDR			0x40024000

typedef enum{
	DAQ_MAX_ERROR_COUNT = 5,
	DAQ_MAX_RUNTIME_TICKS = 5,
	DAQ_MAX_TOTAL_TICKS_ELAPSED = 10
}daq_fault_limits_t;

#endif /* DAQ_DAQ_CONFIG_H_ */
