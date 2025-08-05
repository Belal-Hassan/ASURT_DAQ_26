/*
 * IMU.h
 *
 *  Created on: Jul 17, 2025
 *      Author: Belal
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "DAQ.h"
#include "math.h"
#include "stdlib.h"
#include "string.h"

#define IMU_I2C_ADDRESS DAQ_IMU_I2C_ADDRESS

/** BNO055 ID **/
#define IMU_ID (0xA0)

/*Port and Pin defines*/
#define IMU_RST_GPIO_PORT 	DAQ_IMU_RST_GPIO_PORT
#define IMU_RST_PIN			DAQ_IMU_RST_PIN

#define IMU_EULER_ANGLE_OFFSET_X 0.0 	// Determined experimentally after fixation
#define IMU_EULER_ANGLE_OFFSET_Y 0.0	// Determined experimentally after fixation
  /** BNO055 power settings */
typedef enum
{
  POWER_MODE_NORMAL = 0X00,
  POWER_MODE_LOWPOWER = 0X01,
  POWER_MODE_SUSPEND = 0X02

} imu_powermode_t;

  /** Operation mode settings **/
typedef enum
{
    OPERATION_MODE_CONFIG = 0X00,
    OPERATION_MODE_ACCONLY = 0X01,
    OPERATION_MODE_MAGONLY = 0X02,
    OPERATION_MODE_GYRONLY = 0X03,
    OPERATION_MODE_ACCMAG = 0X04,
    OPERATION_MODE_ACCGYRO = 0X05,
    OPERATION_MODE_MAGGYRO = 0X06,
    OPERATION_MODE_AMG = 0X07,
    OPERATION_MODE_IMUPLUS = 0X08,
    OPERATION_MODE_COMPASS = 0X09,
    OPERATION_MODE_M4G = 0X0A,
    OPERATION_MODE_NDOF_FMC_OFF = 0X0B,
    OPERATION_MODE_NDOF = 0X0C

} imu_opmode_t;

  /** Remap settings **/
typedef enum
{
    REMAP_CONFIG_P0 = 0x21,
    REMAP_CONFIG_P1 = 0x24, // default
    REMAP_CONFIG_P2 = 0x24,
    REMAP_CONFIG_P3 = 0x21,
    REMAP_CONFIG_P4 = 0x24,
    REMAP_CONFIG_P5 = 0x21,
    REMAP_CONFIG_P6 = 0x21,
    REMAP_CONFIG_P7 = 0x24

} imu_axis_remap_config_t;

  /** Remap Signs **/
typedef enum
{
    REMAP_SIGN_P0 = 0x04,
    REMAP_SIGN_P1 = 0x00, // default
    REMAP_SIGN_P2 = 0x06,
    REMAP_SIGN_P3 = 0x02,
    REMAP_SIGN_P4 = 0x03,
    REMAP_SIGN_P5 = 0x01,
    REMAP_SIGN_P6 = 0x07,
    REMAP_SIGN_P7 = 0x05

}imu_axis_remap_sign_t;

  /** Vector Mappings **/
typedef enum
{
    VECTOR_ACCELEROMETER= 0x08,
    VECTOR_MAGNETOMETER	= 0x0E,
    VECTOR_GYROSCOPE 	= 0x14,
    VECTOR_EULER 		= 0x1A,
    VECTOR_LINEARACCEL 	= 0X28,
    VECTOR_GRAVITY 		= 0X2E
} imu_vector_type_t;

/*========================External Structs & Enums========================*/
typedef struct {
  uint8_t x;
  uint8_t x_sign;
  uint8_t y;
  uint8_t y_sign;
  uint8_t z;
  uint8_t z_sign;
} imu_axis_map_t;

typedef enum {
  BNO055_AXIS_SIGN_POSITIVE = 0x00,
  BNO055_AXIS_SIGN_NEGATIVE = 0x01
}imu_axis_map_sign_t;

typedef enum {
  BNO055_AXIS_X = 0x00,
  BNO055_AXIS_Y = 0x01,
  BNO055_AXIS_Z = 0x02
}imu_axis_map_representation_t;

typedef enum {
	BNO055_MAX_ACCELERATION_2G = 0x00,
	BNO055_MAX_ACCELERATION_4G,
	BNO055_MAX_ACCELERATION_8G,
	BNO055_MAX_ACCELERATION_16G
}imu_max_acceleration_t;

typedef struct{
	float x;
	float y;
	float z;
}imu_vector_t;

typedef struct{
	imu_vector_t prev;
	imu_vector_t current;
}imu_reading_t;

void IMU_Init(I2C_HandleTypeDef* hi2c, imu_opmode_t mode, imu_axis_map_t map);
void IMU_Task(void *pvParameters);
void IMU_SetMode(imu_opmode_t mode);
void IMU_GetVector(imu_vector_type_t vector_type , float* xyz);
void IMU_WriteData(uint8_t reg, uint8_t data);
void IMU_SetAxisMap(imu_axis_map_t axis);
void IMU_SelectRegPage(uint8_t page);
void IMU_Eulers_Apply_Offset(imu_vector_t* angles);
void IMU_Transform_Accels(imu_vector_t* accels);


#endif /* INC_IMU_H_ */
