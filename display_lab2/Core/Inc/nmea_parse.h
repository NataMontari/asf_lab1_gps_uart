/**
  ******************************************************************************
  * @file           : nmea_parser.h
  * @brief          : NMEA sentence parser header
  ******************************************************************************
  */

#ifndef NMEA_PARSER_H
#define NMEA_PARSER_H

#include <stdint.h>

// Buffer size definitions
#define DataBuffer_SIZE 512

typedef struct NMEA_DATA {
    double latitude; //latitude in degrees with decimal places
    char latSide;  // N or S
    double longitude; //longitude in degrees with decimal places
    char lonSide; // E or W
    float altitude; //altitude in meters
    float hdop; //horizontal dilution of precision
    int satelliteCount; //number of satellites used in measurement
    int fix; // 1 = fix, 0 = no fix
    char lastMeasure[10]; // hhmmss.ss UTC of last successful measurement; time read from the GPS module
} GPS;

// Function prototypes
int gps_checksum(char *nmea_data);
int nmea_GPGLL(GPS *gps_data, char *inputString);
int nmea_GPGSA(GPS *gps_data, char *inputString);
int nmea_GPGGA(GPS *gps_data, char *inputString);
void nmea_parse(GPS *gps_data, uint8_t *buffer);

// External variable for raw mode (defined in main.c)
extern uint8_t raw_mode_on;

#endif /* NMEA_PARSER_H */
