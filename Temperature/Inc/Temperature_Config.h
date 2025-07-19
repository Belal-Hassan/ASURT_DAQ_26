/*
 * Temperature_Config.h
 *
 *  Created on: Jul 18, 2025
 *      Author: Belal
 */

#ifndef INC_TEMPERATURE_CONFIG_H_
#define INC_TEMPERATURE_CONFIG_H_


#define MLX90614_ADDR  		0x5A  // 8-bit address format for STM32 HAL
#define I2C_ADDR_MULTIPLIER 1     // Subjected to change
// Define register addresses
//#define MLX90614_TA    0x06  	// Ambient temperature register
#define MLX90614_TOBJ  0x07  	// Object temperature register
// Moving average parameters
#define MOVING_AVG_WINDOW_SIZE 		5


#endif /* INC_TEMPERATURE_CONFIG_H_ */
