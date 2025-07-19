/*
 * IMU.c
 *
 *  Created on: Jul 17, 2025
 *      Author: Belal
 */

#include "IMU.h"

extern SemaphoreHandle_t i2c_mutex;
imu_readings_t imu_lin_accelerations, imu_euler_angles;

void IMU_Init(I2C_HandleTypeDef* hi2c, adafruit_bno055_opmode_t mode, adafruit_bno055_axis_map_t map)
{
	Adafruit_BNO055_Init(hi2c, 123, BNO055_ADDRESS);
	Adafruit_BNO055_Begin(mode);
	Adafruit_BNO055_SetAxisMap(map);
}
void IMU_Task(void*pvParameters)
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	for( ;; )
	{
		if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
		{
			Adafruit_BNO055_GetVector(VECTOR_EULER, (double*)&imu_euler_angles.current);
			Adafruit_BNO055_GetVector(VECTOR_LINEARACCEL, (double*)&imu_lin_accelerations.current);
			xSemaphoreGive(i2c_mutex);
		}
		if(DAQ_Check_Change(imu_euler_angles.current.x, imu_euler_angles.prev.x, DAQ_MIN_CHANGE_IMU_ANGLE_X) ||
		   DAQ_Check_Change(imu_euler_angles.current.y, imu_euler_angles.prev.y, DAQ_MIN_CHANGE_IMU_ANGLE_Y) ||
		   DAQ_Check_Change(imu_euler_angles.current.z, imu_euler_angles.prev.z, DAQ_MIN_CHANGE_IMU_ANGLE_Z))
		{
			daq_can_msg_t can_msg_imu_angle = {};
			daq_can_msg_imu_t encoder_msg_imu_angle = {};
			encoder_msg_imu_angle.x = (int16_t)(imu_euler_angles.current.x * DAQ_ACCURACY_IMU_ANGLE_X);
			encoder_msg_imu_angle.y = (int16_t)(imu_euler_angles.current.y * DAQ_ACCURACY_IMU_ANGLE_Y);
			encoder_msg_imu_angle.z = (int16_t)(imu_euler_angles.current.z * DAQ_ACCURACY_IMU_ANGLE_Z);
			can_msg_imu_angle.id = DAQ_CAN_ID_IMU;
			can_msg_imu_angle.size = 8;
			can_msg_imu_angle.data = *((uint64_t*)(&encoder_msg_imu_angle));
			DAQ_CAN_Msg_Enqueue(&can_msg_imu_angle);
			imu_euler_angles.prev.x = imu_euler_angles.current.x;
			imu_euler_angles.prev.y = imu_euler_angles.current.y;
			imu_euler_angles.prev.z = imu_euler_angles.current.z;
		}
		vTaskDelayUntil(&xLastWakeTime, 11);
	}
}
