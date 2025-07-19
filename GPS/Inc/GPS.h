/*
 * GPS.h
 *
 *  Created on: Jul 1, 2025
 *      Author: Belal
 */

#ifndef GPS_INC_GPS_H_
#define GPS_INC_GPS_H_

#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include "DAQ.h"

#define UBLOX_I2C_ADDRESS (0x42 << 1)
typedef struct {
    char time[11];      // HHMMSS.ss
    char status;        // A = valid, V = invalid
    double latitude;    // Degrees
    char lat_dir;       // N/S
    double longitude;   // Degrees
    char lon_dir;       // E/W
    //double speed;       // Knots
    //char date[7];       // DDMMYY
} gps_gnrmc_data_t;
void GPS_Init(I2C_HandleTypeDef *hi2c);
void GPS_Read_GNRMC(I2C_HandleTypeDef *hi2c);
void GPS_Parse_GNRMC(gps_gnrmc_data_t *data);
void GPS_Task(void * pvParameters);
//void GPS_sendGPSData(UART_HandleTypeDef *huart, GNRMC_Data *data);
//void sendUART(UART_HandleTypeDef *huart, const char *message) ;
double convertToDegrees(char *rawValue);
double my_atof(const char *str);


#endif /* GPS_INC_GPS_H_ */
