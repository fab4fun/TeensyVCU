// gps.cpp
#include "gps.h"

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

void initializeGPS() {
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

 // delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
    // Initialization code here
}

void readGPSData() {
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
}

uint8_t GetGPS_Cnt_Year() {
  return GPS.year;
}

uint8_t GetGPS_Cnt_Month() {
  return GPS.month;
}

uint8_t GetGPS_Cnt_Day() {
  return GPS.day;
}

uint8_t GetGPS_t_Hour() {
  return GPS.hour;
}

uint8_t GetGPS_t_Minute() {
  return GPS.minute;
}

uint8_t GetGPS_t_Seconds() {
  return GPS.seconds;
}

uint16_t GetGPS_t_Milliseconds() {
  return GPS.milliseconds;
}

boolean GetGPS_b_Fix() {
  return GPS.fix;
}

char GetGPS_str_LatPosDir() {
  return GPS.lat;
}

char GetGPS_str_LongPosDir() {
  return GPS.lon;
}

uint8_t GetGPS_e_FixQuality() {
  return GPS.fixquality;
}

float_t GetGPS_deg_LatPos() {
  return GPS.latitude;
}

float_t GetGPS_deg_LongPos() {
    return GPS.longitude;
}

float_t GetGPS_v_Speed() {
    return (GPS.speed*(nmea_float_t)0.514444);
}

float_t GetGPS_deg_Heading() {
    return GPS.angle;
}

float_t GetGPS_l_Altitude() {
    return GPS.altitude;
}

uint8_t GetGPS_Cnt_Satellites() {
    return GPS.satellites;
}

uint8_t GetGPS_e_Antenna() {
    return GPS.antenna;
}

float_t GetGPS_t_SinceFix() {
    return GPS.secondsSinceFix();
}
// gps.cpp