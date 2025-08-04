/*
 * IMU.c
 *
 *  Created on: Jul 17, 2025
 *      Author: Belal
 */

#include "IMU.h"
#include "IMU_Private.h"
#include "wwdg.h"

#ifndef PI
#define PI 					3.14159265359
#endif /* PI */

extern SemaphoreHandle_t g_i2c_mutex;
extern daq_fault_record_t g_fault_record;
imu_readings_t imu_linear_accels, imu_euler_angles;
static imu_opmode_t imu_mode;

/*pointer to save sensor I2c configrations prameters*/
static I2C_HandleTypeDef* GlobalConfig = NULL;

void IMU_SetMode(imu_opmode_t mode)
{
  imu_mode = mode;
  HAL_I2C_Mem_Write(GlobalConfig , IMU_I2C_ADDRESS , BNO055_OPR_MODE_ADDR , 1 , &imu_mode , 1 , 20);
  HAL_Delay(30);
}

void IMU_GetVector(imu_vector_type_t vector_type , float* xyz)
{

  uint8_t buffer[6];
  memset(buffer, 0, 6);

  int16_t x, y, z;
  x = y = z = 0;

  /* Read vector data (6 bytes) */
  HAL_I2C_Mem_Read(GlobalConfig , IMU_I2C_ADDRESS , (adafruit_bno055_reg_t)vector_type , 1 , buffer , 6 , 20);

  x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
  y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
  z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

  /*!
   * Convert the value to an appropriate range (section 3.6.4)
   * and assign the value to the Vector type
   */
  switch (vector_type)
  {
  case VECTOR_MAGNETOMETER:
    /* 1uT = 16 LSB */
    xyz[0] = ((float)x) / 16.0;
    xyz[1] = ((float)y) / 16.0;
    xyz[2] = ((float)z) / 16.0;
    break;
  case VECTOR_GYROSCOPE:
    /* 1dps = 16 LSB */
    xyz[0] = ((float)x) / 16.0;
    xyz[1] = ((float)y) / 16.0;
    xyz[2] = ((float)z) / 16.0;
    break;
  case VECTOR_EULER:
    /* 1 degree = 16 LSB */
    xyz[0] = ((float)x) / 16.0;
    xyz[1] = ((float)y) / 16.0;
    xyz[2] = ((float)z) / 16.0;
    break;
  case VECTOR_ACCELEROMETER:
    /* 1m/s^2 = 100 LSB */
    xyz[0] = ((float)x) / 100.0;
    xyz[1] = ((float)y) / 100.0;
    xyz[2] = ((float)z) / 100.0;
    break;
  case VECTOR_LINEARACCEL:
    /* 1m/s^2 = 100 LSB */
    xyz[0] = ((float)x) / 100.0;
    xyz[1] = ((float)y) / 100.0;
    xyz[2] = ((float)z) / 100.0;
    break;
  case VECTOR_GRAVITY:
    /* 1m/s^2 = 100 LSB */
    xyz[0] = ((float)x) / 100.0;
    xyz[1] = ((float)y) / 100.0;
    xyz[2] = ((float)z) / 100.0;
    break;
  }
}

/*========================External Functions========================*/
void IMU_WriteData(uint8_t reg, uint8_t data)
{
	HAL_I2C_Mem_Write(GlobalConfig , IMU_I2C_ADDRESS , reg , 1 , &data , 1 , 20);
}
void IMU_SetAxisMap(imu_axis_map_t axis)
{
	uint8_t axisRemap = (axis.z << 4) | (axis.y << 2) | (axis.x);
	uint8_t axisMapSign = (axis.x_sign << 2) | (axis.y_sign << 1) | (axis.z_sign);
	IMU_WriteData(BNO055_AXIS_MAP_CONFIG_ADDR, axisRemap);
	IMU_WriteData(BNO055_AXIS_MAP_SIGN_ADDR, axisMapSign);
}

void IMU_SelectRegPage(uint8_t page)
{
	if(page == 0 || page == 1)
		IMU_WriteData(BNO055_PAGE_ID_ADDR, page);
	HAL_Delay(2);
}
void IMU_Init(I2C_HandleTypeDef* hi2c, imu_opmode_t mode, imu_axis_map_t map)
{
	GlobalConfig = hi2c;
	uint8_t Data = 0;
	uint8_t id;
	HAL_StatusTypeDef status;

	// Reset the sensor first
	Data = 0x20;
	status = HAL_I2C_Mem_Write(GlobalConfig, IMU_I2C_ADDRESS, BNO055_SYS_TRIGGER_ADDR, 1, &Data, 1, 20);
	if (status != HAL_OK)
		return;
	//HAL_Delay(650); // Per datasheet, reset takes ~650ms
	for(uint8_t i = 0; i < 13; i++)
	{
		HAL_Delay(50);
		HAL_WWDG_Refresh(&hwwdg);
	}
	// Check device ID
	status = HAL_I2C_Mem_Read(GlobalConfig, IMU_I2C_ADDRESS, BNO055_CHIP_ID_ADDR, 1, &id, 1, 20);
	if (status != HAL_OK || id != IMU_ID)
		return;

	// Set to normal power mode
	IMU_WriteData(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
	HAL_Delay(10);

	// Switch to register page 1
	IMU_SelectRegPage(1);
	// Set acceleration max to 16g.
	IMU_WriteData(ACCEL_CONFIG, BNO055_MAX_ACCELERATION_16G);
	HAL_Delay(10);
	// Switch to register page 0
	IMU_SelectRegPage(0);

	// Remap axes
	IMU_SetAxisMap(map);

	// Set operating mode
	IMU_SetMode(mode);
	HAL_Delay(20);
	HAL_WWDG_Refresh(&hwwdg);
}
void IMU_Task(void*pvParameters)
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	for( ;; )
	{
		g_fault_record.tasks[IMU_TASK].start_tick = xTaskGetTickCount();
		if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
		{
//			IMU_GetVector(VECTOR_EULER, (float*)&imu_euler_angles.current);
//			IMU_GetVector(VECTOR_LINEARACCEL, (float*)&imu_linear_accels.current);
			xSemaphoreGive(g_i2c_mutex);
			imu_euler_angles.current.x -= IMU_EULER_ANGLE_OFFSET_X;
			imu_euler_angles.current.y -= IMU_EULER_ANGLE_OFFSET_Y;
		}
//		if(DAQ_CheckChange(imu_euler_angles.current.x, imu_euler_angles.prev.x, DAQ_MIN_CHANGE_IMU_ANGLE_X) ||
//		   DAQ_CheckChange(imu_euler_angles.current.y, imu_euler_angles.prev.y, DAQ_MIN_CHANGE_IMU_ANGLE_Y) ||
//		   DAQ_CheckChange(imu_euler_angles.current.z, imu_euler_angles.prev.z, DAQ_MIN_CHANGE_IMU_ANGLE_Z))
		{
			daq_can_msg_t can_msg_imu_angle = {0};
			daq_can_msg_imu_t encoder_msg_imu_angle = {0};
			encoder_msg_imu_angle.x = (int16_t)(imu_euler_angles.current.x * DAQ_ACCURACY_IMU_ANGLE_X);
			encoder_msg_imu_angle.y = (int16_t)(imu_euler_angles.current.y * DAQ_ACCURACY_IMU_ANGLE_Y);
			encoder_msg_imu_angle.z = (int16_t)(imu_euler_angles.current.z * DAQ_ACCURACY_IMU_ANGLE_Z);
			can_msg_imu_angle.id = DAQ_CAN_ID_IMU_ANGLE;
			can_msg_imu_angle.size = 8;
			can_msg_imu_angle.data = *((uint64_t*)(&encoder_msg_imu_angle));
			DAQ_CAN_Msg_Enqueue(&can_msg_imu_angle);
			imu_euler_angles.prev.x = imu_euler_angles.current.x;
			imu_euler_angles.prev.y = imu_euler_angles.current.y;
			imu_euler_angles.prev.z = imu_euler_angles.current.z;
		}
//		if(DAQ_CheckChange(imu_linear_accels.current.x, imu_linear_accels.prev.x, DAQ_MIN_CHANGE_IMU_ACCEL) ||
//		   DAQ_CheckChange(imu_linear_accels.current.y, imu_linear_accels.prev.y, DAQ_MIN_CHANGE_IMU_ACCEL) ||
//		   DAQ_CheckChange(imu_linear_accels.current.z, imu_linear_accels.prev.z, DAQ_MIN_CHANGE_IMU_ACCEL))
		{
			daq_can_msg_t can_msg_imu_acceleration = {0};
			daq_can_msg_imu_t encoder_msg_imu_acceleration = {0};
			encoder_msg_imu_acceleration.x = (int16_t)(imu_linear_accels.current.x * DAQ_ACCURACY_IMU_ANGLE_X);
			encoder_msg_imu_acceleration.y = (int16_t)(imu_linear_accels.current.y * DAQ_ACCURACY_IMU_ANGLE_Y);
			encoder_msg_imu_acceleration.z = (int16_t)(imu_linear_accels.current.z * DAQ_ACCURACY_IMU_ANGLE_Z);
			can_msg_imu_acceleration.id = DAQ_CAN_ID_IMU_ACCEL;
			can_msg_imu_acceleration.size = 8;
			can_msg_imu_acceleration.data = *((uint64_t*)(&encoder_msg_imu_acceleration));
			DAQ_CAN_Msg_Enqueue(&can_msg_imu_acceleration);
			imu_linear_accels.prev.x = imu_linear_accels.current.x;
			imu_linear_accels.prev.y = imu_linear_accels.current.y;
			imu_linear_accels.prev.z = imu_linear_accels.current.z;
		}
		//for(uint64_t i = 0; i < 6000000; i++);
		g_fault_record.tasks[IMU_TASK].entry_count++;
		if(g_fault_record.tasks[IMU_TASK].runtime == 0)
		{
			g_fault_record.tasks[IMU_TASK].runtime = xTaskGetTickCount();
			g_fault_record.tasks[IMU_TASK].runtime -= g_fault_record.tasks[IMU_TASK].start_tick;
		}
		vTaskDelayUntil(&xLastWakeTime, 8);
	}
}
