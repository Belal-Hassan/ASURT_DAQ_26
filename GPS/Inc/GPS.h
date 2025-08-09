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

#define GPS_I2C_ADDRESS DAQ_GPS_I2C_ADDRESS

typedef struct {
    char time[11];      // HHMMSS.ss
    char status;        // A = valid, V = invalid
    double latitude;    // Degrees
    char lat_dir;       // N/S
    double longitude;   // Degrees
    char lon_dir;       // E/W
} gps_gnrmc_data_t;

void GPS_Init(I2C_HandleTypeDef *hi2c);
void GPS_ReadGNRMC(I2C_HandleTypeDef *hi2c);
void GPS_ParseGNRMC(gps_gnrmc_data_t *data);
void GPS_Task(void * pvParameters);


#endif /* GPS_INC_GPS_H_ */
