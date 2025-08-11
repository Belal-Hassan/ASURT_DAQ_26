/*
 * GPS.h
 *
 *  Created on: Jul 1, 2025
 *      Author: Belal
 */

#ifndef GPS_INC_GPS_H_
#define GPS_INC_GPS_H_

#include "main.h"
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include "DAQ.h"
/**
 * @defgroup GPS_Module GPS Group
 * @brief Functions and data structures for GPS communication and data parsing.
 *
 * This module provides initialization, reading, and parsing of GPS data,
 * specifically handling GNRMC sentences over I2C.
 *
 * @{
 */

/**
 * @brief I2C address for the GPS device.
 */
#define GPS_I2C_ADDRESS DAQ_GPS_I2C_ADDRESS

/**
 * @brief Structure to hold parsed GNRMC GPS data.
 *
 * @note There are more entries but aren't needed in this implementation.
 */
typedef struct {
    char time[11];      /*!< UTC time in "HHMMSS.ss" format. */
    char status;        /*!< 'A' if data is valid, 'V' if invalid. */
    double latitude;    /*!< Latitude in decimal degrees. */
    char lat_dir;       /*!< 'N' for North or 'S' for South. */
    double longitude;   /*!< Longitude in decimal degrees. */
    char lon_dir;       /*!< 'E' for East or 'W' for West. */
} gps_gnrmc_data_t;
/**
 * @brief Initializes the GPS module.
 * @param hi2c Pointer to the I2C handle used for communication with GPS.
 */
void GPS_Init(I2C_HandleTypeDef *hi2c);
/**
 * @brief Initiates an I2C read of the GNRMC sentence from the GPS device.
 * Uses interrupt-driven receive if the I2C DMA is free.
 * @param hi2c Pointer to I2C handle used for communication.
 */
void GPS_ReadGNRMC(I2C_HandleTypeDef *hi2c);
/**
 * @brief Parses the received GNRMC GPS data from the I2C buffer.
 * Fills the gps_gnrmc_data_t struct with parsed values.
 * Ignores invalid data indicated by status 'V'.
 * @param data Pointer to the structure to fill with parsed GPS data.
 */
void GPS_ParseGNRMC(gps_gnrmc_data_t *data);
/**
 * @brief FreeRTOS task that periodically reads and processes GPS data.
 *
 * Takes the I2C mutex before reading GPS data to avoid conflicts.
 * When new valid GPS data is received, encodes and enqueues a CAN message.
 * Updates fault records with timing and execution counts.
 *
 * @param pvParameters Task parameter, unused.
 */
void GPS_Task(void * pvParameters);
/** @}*/


#endif /* GPS_INC_GPS_H_ */
