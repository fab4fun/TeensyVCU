// gps.h
#ifndef GPS_H
#define GPS_H

#include <Adafruit_GPS.h>
#include <Arduino.h>

// what's the name of the hardware serial port?
#define GPSSerial Serial8



// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

void initializeGPS();
void readGPSData();

uint8_t GetGPS_Cnt_Year();
uint8_t GetGPS_Cnt_Month();
uint8_t GetGPS_Cnt_Day();

uint8_t GetGPS_t_Hour();
uint8_t GetGPS_t_Minute();
uint8_t GetGPS_t_Seconds();
uint16_t GetGPS_t_Milliseconds();
boolean GetGPS_b_Fix();
char GetGPS_str_LatPosDir();
char GetGPS_str_LongPosDir();
uint8_t GetGPS_e_FixQuality();
float_t GetGPS_deg_LatPos();
float_t GetGPS_deg_LongPos();
float_t GetGPS_v_Speed();
float_t GetGPS_deg_Heading();
float_t GetGPS_l_Altitude();

uint8_t GetGPS_Cnt_Satellites();
uint8_t GetGPS_e_Antenna();

float_t GetGPS_t_SinceFix();

#endif // GPS_H