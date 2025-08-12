/*
 * IMU.h
 *
 *  Created on: Jul 17, 2025
 *      Author: Belal
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "main.h"
#include "DAQ.h"
#include "math.h"
#include "stdlib.h"
#include "string.h"

/**
 * @addtogroup IMU_Module
 * @{
 */

/**
 * @def IMU_I2C_ADDRESS
 * @brief I2C address of the IMU sensor (typically 0x28 or 0x29)
 *
 * Configured through DAQ_IMU_I2C_ADDRESS in system configuration.
 */
#define IMU_I2C_ADDRESS DAQ_IMU_I2C_ADDRESS

/**
 * @def IMU_ID
 * @brief Expected chip ID for BNO055 sensor (0xA0)
 *
 * Used during initialization to verify proper communication with the sensor.
 */
#define IMU_ID (0xA0)

/**
 * @def IMU_RST_GPIO_PORT
 * @def IMU_RST_PIN
 * @brief Hardware reset pin configuration for the IMU
 *
 * Allows software-controlled reset of the IMU sensor when needed.
 */
#define IMU_RST_GPIO_PORT 	DAQ_IMU_RST_GPIO_PORT
#define IMU_RST_PIN			DAQ_IMU_RST_PIN

/**
 * @brief Mounting offset corrections for x axis Euler angles
 *
 * It compensate for imperfect sensor mounting alignment.
 * Determined experimentally after physical installation.
 */
#define IMU_EULER_ANGLE_OFFSET_X 0.0
/**
 * @brief Mounting offset corrections for y-axis Euler angles
 *
 * It compensate for imperfect sensor mounting alignment.
 * Determined experimentally after physical installation.
 */
#define IMU_EULER_ANGLE_OFFSET_Y 0.0

/**
 * @brief Power modes for the BNO055 sensor
 */
typedef enum
{
  POWER_MODE_NORMAL = 0X00,   ///< Normal operation mode (full performance)
  POWER_MODE_LOWPOWER = 0X01, ///< Reduced power consumption mode
  POWER_MODE_SUSPEND = 0X02   ///< Lowest power mode (sensor disabled)
} imu_powermode_t;

/**
 * @brief Operational modes for sensor fusion and data processing
 *
 * The BNO055 offers multiple modes that determine which sensors are active
 * and how data is processed/fused. NDOF (Nine Degrees of Freedom) mode
 * provides the most complete orientation data.
 */
typedef enum
{
    OPERATION_MODE_CONFIG = 0X00,       ///< Configuration mode (change settings)
    OPERATION_MODE_ACCONLY = 0X01,      ///< Accelerometer only
    OPERATION_MODE_MAGONLY = 0X02,      ///< Magnetometer only
    OPERATION_MODE_GYRONLY = 0X03,      ///< Gyroscope only
    OPERATION_MODE_ACCMAG = 0X04,       ///< Accelerometer + Magnetometer
    OPERATION_MODE_ACCGYRO = 0X05,      ///< Accelerometer + Gyroscope
    OPERATION_MODE_MAGGYRO = 0X06,      ///< Magnetometer + Gyroscope
    OPERATION_MODE_AMG = 0X07,          ///< Accelerometer + Magnetometer + Gyroscope (no fusion)
    OPERATION_MODE_IMUPLUS = 0X08,      ///< IMU mode (accelerometer + gyroscope fusion)
    OPERATION_MODE_COMPASS = 0X09,      ///< Compass mode (accelerometer + magnetometer fusion)
    OPERATION_MODE_M4G = 0X0A,          ///< M4G mode (magnetometer + gyroscope fusion)
    OPERATION_MODE_NDOF_FMC_OFF = 0X0B, ///< NDOF mode with fast magnetic calibration off
    OPERATION_MODE_NDOF = 0X0C          ///< NDOF mode (full 9-DoF with sensor fusion)
} imu_opmode_t;

/**
 * @enum imu_vector_type_t
 * @brief Types of sensor data vectors that can be read
 *
 * Each vector type has a specific register address and conversion factor.
 */
typedef enum
{
    VECTOR_ACCELEROMETER = 0x08,   ///< Acceleration data (m/s²)
    VECTOR_MAGNETOMETER  = 0x0E,   ///< Magnetic field data (μT)
    VECTOR_GYROSCOPE     = 0x14,   ///< Angular velocity data (dps)
    VECTOR_EULER         = 0x1A,   ///< Euler angles (degrees)
    VECTOR_LINEARACCEL   = 0X28,   ///< Linear acceleration (without gravity) (m/s²)
    VECTOR_GRAVITY       = 0X2E    ///< Gravity vector (m/s²)
} imu_vector_type_t;

/**
 * @struct imu_axis_map_t
 * @brief Complete axis mapping configuration
 *
 * Defines both the physical axis remapping and sign configuration.
 * Used to adapt the sensor to different mounting orientations.
 */
typedef struct {
  uint8_t x;           ///< Remapped X axis (BNO055_AXIS_X/Y/Z)
  uint8_t x_sign;      ///< X axis sign (BNO055_AXIS_SIGN_POSITIVE/NEGATIVE)
  uint8_t y;           ///< Remapped Y axis
  uint8_t y_sign;      ///< Y axis sign
  uint8_t z;           ///< Remapped Z axis
  uint8_t z_sign;      ///< Z axis sign
} imu_axis_map_t;

/**
 * @enum imu_axis_map_sign_t
 * @brief Axis sign options for mapping
 */
typedef enum {
  BNO055_AXIS_SIGN_POSITIVE = 0x00, ///< Positive direction
  BNO055_AXIS_SIGN_NEGATIVE = 0x01  ///< Negative direction
}imu_axis_map_sign_t;

/**
 * @enum imu_axis_map_representation_t
 * @brief Physical axis identifiers for mapping
 */
typedef enum {
  BNO055_AXIS_X = 0x00, ///< X axis
  BNO055_AXIS_Y = 0x01, ///< Y axis
  BNO055_AXIS_Z = 0x02  ///< Z axis
}imu_axis_map_representation_t;

/**
 * @brief Maximum acceleration range settings
 *
 * Configures the accelerometer's measurement range.
 */
typedef enum {
	BNO055_MAX_ACCELERATION_2G = 0x00, ///< ±2g range
	BNO055_MAX_ACCELERATION_4G,        ///< ±4g range
	BNO055_MAX_ACCELERATION_8G,        ///< ±8g range
	BNO055_MAX_ACCELERATION_16G        ///< ±16g range
}imu_max_acceleration_t;

/**
 * @brief 3D vector representation for sensor data
 */
typedef struct{
	float x; ///< X component of the vector (euler, accleration, etc.)
	float y; ///< Y component of the vector (euler, accleration, etc.)
	float z; ///< Z component of the vector (euler, accleration, etc.)
}imu_vector_t;

/**
 * @brief Buffer for tracking previous and current sensor readings
 *
 * Used to detect meaningful changes in sensor data before transmission.
 */
typedef struct{
	imu_vector_t prev;    ///< Previous readings
	imu_vector_t current; ///< Current readings
}imu_reading_buffer_t;

/**
 * @brief Initialize the IMU sensor with specified configuration
 *
 * Performs complete initialization sequence:
 * - Hardware reset
 * - Device ID verification
 * - Power mode configuration
 * - Axis remapping
 * - Operation mode setup
 *
 * @param hi2c I2C handle for communication
 * @param mode Desired operation mode (typically OPERATION_MODE_NDOF)
 * @param map Axis mapping configuration for physical orientation
 */
void IMU_Init(I2C_HandleTypeDef* hi2c, imu_opmode_t mode, imu_axis_map_t map);

/**
 * @brief Main task for IMU data processing and transmission
 *
 * Periodically reads sensor data, applies corrections, and transmits
 * via CAN when significant changes are detected.
 *
 * @param pvParameters Unused task parameter (required by FreeRTOS)
 */
void IMU_Task(void *pvParameters);

/**
 * @brief Change the IMU's operation mode
 *
 * Switches between different sensor fusion modes. Requires transitioning
 * through CONFIG mode first. Includes necessary delays for mode changes.
 *
 * @param mode Target operation mode
 */
void IMU_SetMode(imu_opmode_t mode);

/**
 * @brief Read a specific vector type from the IMU
 *
 * Retrieves and converts raw sensor data to physical units:
 * - Accelerometer: m/s² (100 LSB = 1 m/s²)
 * - Gyroscope: degrees/second (16 LSB = 1 dps)
 * - Magnetometer: microtesla (16 LSB = 1 μT)
 * - Euler angles: degrees (16 LSB = 1 degree)
 *
 * @param vector_type Type of vector to read
 * @param xyz Array to store the X, Y, Z values (3 elements)
 */
void IMU_GetVector(imu_vector_type_t vector_type , float* xyz);

/**
 * @brief Write a single byte to an IMU register
 *
 * Low-level register access for configuration.
 *
 * @param reg Register address
 * @param data Value to write
 */
void IMU_WriteData(uint8_t reg, uint8_t data);

/**
 * @brief Configure the axis mapping
 *
 * Sets both the physical axis remapping and sign configuration to adapt
 * the sensor to its physical mounting orientation.
 *
 * @param axis Axis mapping configuration
 */
void IMU_SetAxisMap(imu_axis_map_t axis);

/**
 * @brief Select register page (0 or 1)
 *
 * The BNO055 uses a paged register map. Most configuration requires
 * accessing registers on page 1.
 *
 * @param page Register page (0 or 1)
 */
void IMU_SelectRegPage(uint8_t page);

/**
 * @brief Apply mounting offset corrections to Euler angles
 *
 * Adjusts angle readings based on experimentally determined mounting offsets.
 *
 * @param angles Pointer to angle vector to correct
 */
void IMU_Eulers_Apply_Offset(imu_vector_t* angles);

/**
 * @brief Transform acceleration vectors based on mounting orientation
 *
 * Applies a rotation matrix to acceleration data to account for non-ideal
 * sensor mounting angles. Ensures acceleration data aligns with vehicle axes.
 *
 * @param accels Pointer to acceleration vector to transform
 */
void IMU_Transform_Accels(imu_vector_t* accels);

/** @} */

#endif /* INC_IMU_H_ */
