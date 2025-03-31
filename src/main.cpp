#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "TeensyDebug.h"
#pragma GCC optimize ("O0")

// Test code for Ultimate GPS Using Hardware Serial (e.g. GPS Flora or FeatherWing)
//
// This code shows how to listen to the GPS module via polling. Best used with
// Feathers or Flora where you have hardware Serial and no interrupt
//
// Tested and works great with the Adafruit GPS FeatherWing
// ------> https://www.adafruit.com/products/3133
// or Flora GPS
// ------> https://www.adafruit.com/products/1059
// but also works with the shield, breakout
// ------> https://www.adafruit.com/products/1272
// ------> https://www.adafruit.com/products/746
//
// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software! -ada



#include "sdFunc.h"
#include "gps.h"
#include "wiring.h"

//#include "tasks.h"  

extern void MngTASK_Loop(void);
extern void MngTASK_Init(void);


uint32_t timer = millis();

//------------------------------------------------------------------------------
// File system object.
SdFat sd;

// Log file.
SdFile file;

// Time in micros for next data record.
uint32_t logTime;

void setup()
{
  //while (!Serial);  // uncomment to have the sketch wait until Serial is ready

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic parsing test!");

// Debugger will use second USB Serial; this line is not need if using menu option
  debug.begin(SerialUSB1);

  //halt_cpu();

  MngTASK_Init();
  initializeGPS();

  // SD_setup();

  // switched 12V digital input pin
  pinMode(12, INPUT);

  // set power hold pin high until ready to shutdown
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  //GPS enable pin
  pinMode(37, OUTPUT);
  // set low to disable GPS for now
  digitalWrite(37, LOW);

}

void loop() // run over and over again
{
  MngTASK_Loop();

  readGPSData();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    Serial.print("\nTime: ");
    if (GetGPS_t_Hour() < 10) {
      Serial.print('0');
    }
    Serial.print(GetGPS_t_Hour(), DEC); Serial.print(':');
    if (GetGPS_t_Minute() < 10) { Serial.print('0'); }
    Serial.print(GetGPS_t_Minute(), DEC); Serial.print(':');
    if (GetGPS_t_Seconds() < 10) { Serial.print('0'); }
    Serial.print(GetGPS_t_Seconds(), DEC); Serial.print('.');
    if (GetGPS_t_Milliseconds() < 10) {
      Serial.print("00");
    }
    else if (GetGPS_t_Milliseconds() > 9 && GetGPS_t_Milliseconds()< 100) {
      Serial.print("0");
    }
    Serial.println(GetGPS_t_Milliseconds());
    Serial.print("Date: ");
    Serial.print(GetGPS_Cnt_Day(), DEC); Serial.print('/');
    Serial.print(GetGPS_Cnt_Month(), DEC); Serial.print("/20");
    Serial.println(GetGPS_Cnt_Year(), DEC);
    Serial.print("Fix quality: ");  Serial.println(GetGPS_e_FixQuality());
    if (GetGPS_b_Fix()) {
      Serial.println("Fix"); 
      Serial.print("Location: ");
      Serial.print(GetGPS_deg_LatPos(), 4); Serial.print(GetGPS_str_LatPosDir());
      Serial.print(", ");
      Serial.print(GetGPS_deg_LatPos(), 4); Serial.print(GetGPS_str_LongPosDir());
      Serial.print("Speed (m/s): ");
      Serial.println(GetGPS_v_Speed());
      Serial.print("Heading: ");
      Serial.println(GetGPS_deg_Heading());
      Serial.print("Altitude: "); Serial.println(GetGPS_l_Altitude());
      Serial.print("Satellites: "); Serial.println((int)GetGPS_Cnt_Satellites());
      Serial.print("Antenna status: "); Serial.println((int)GetGPS_e_Antenna());
    }
    else {
      Serial.println("No Fix"); 
    }
  }
  // Time for next record.
  logTime += 1000UL*SAMPLE_INTERVAL_MS;

  // Wait for log time.
 /* int32_t diff;
  do {
    diff = micros() - logTime;
  } while (diff < 0); */

  // Check for data rate too high.
//  if (diff > 10) {
//    error("Missed data record");
//  }

 // logData();

  // Force data to SD and update the directory entry to avoid data loss.
/*  if (!file.sync() || file.getWriteError()) {
    error("write error");
  } */

 /* if (Serial.available()) {
    // Close file and stop.
    file.close();
    Serial.println(F("Done"));
    while (true) {}
  } */
}


