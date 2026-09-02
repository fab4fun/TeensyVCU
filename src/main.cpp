// Added stdbool header to satisfy dependencies for Arduino.h
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
//#include "TeensyDebug.h"
#pragma GCC optimize ("O0")

//#include "sdFunc.h"
#include "gps.h"
#include "ThrottleCtrl.h"
//#include "wiring.h"
//#include "tasks.h"  

extern void MngTASK_Loop(void);
extern void MngTASK_Init(void);

// Set to 1 if the GPS module is physically installed on this VCU build; 0 for bench/units without one.
// Gates all GPS-derived periodic debug text (Time/Date/Fix/Location). The throttle position/current/fault
// line stays printed regardless so bench testing still has visibility.
#define GPS_INSTALLED 0

uint32_t timer = millis();

void setup()
{
  //halt_cpu();

  MngTASK_Init();

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

  // NOTE: the previous standalone parseFloat() target override was removed because it ran
  // BEFORE the line-based parser and consumed every incoming byte first — so typing "FWD 30"
  // got parsed as a number (0), its drain loop ate the rest of the command, and open-loop
  // commands never reached the real parser. All bench input now flows through the single
  // line-based parser below (plain numbers still work as closed-loop targets).

  // Bench-test input (Enter-terminated, non-blocking):
  //   FWD <duty> -> open-loop drive HB1 direction at <duty>% (bypasses PID + fault logic)
  //   REV <duty> -> open-loop drive HB2 direction at <duty>%
  //   STOP       -> exit open-loop mode, hold motor off
  //   <number>   -> closed-loop target position % (exits open-loop mode), e.g. "40"
  static String benchCmd = "";
  if (Serial.available()) {
    char c = Serial.read();
    // Local echo: the serial monitor does not reliably show typed characters, so echo each
    // received byte back. On a line terminator also print a newline so the command you just
    // submitted is clearly delimited from the periodic status text that follows.
    if (c == '\n' || c == '\r') {
      if (benchCmd.length() > 0) {
        Serial.print("cmd: ");
        Serial.println(benchCmd);
        if (benchCmd.startsWith("FWD")) SetThrottleOpenLoop(1, benchCmd.substring(4).toInt());
        else if (benchCmd.startsWith("REV")) SetThrottleOpenLoop(-1, benchCmd.substring(4).toInt());
        else if (benchCmd == "STOP") ExitThrottleOpenLoop();
        else { float pct = benchCmd.toFloat(); ExitThrottleOpenLoop(); SetThrottleTargetPct(pct); }
      }
      Serial.println(); // newline so the echoed command sits on its own line
      benchCmd = "";
    } else {
      if (c != '\r' && c != '\n') Serial.write(c); // echo printable characters back to you
      benchCmd += c;
    }
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    if (GPS_INSTALLED) {
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
    else {
      Serial.println(); // blank separator so the throttle line below is easy to spot
    }
    Serial.print("TPS1: "); Serial.print(GetTPS1Volts(), 3); Serial.print("V ("); Serial.print(GetTPS1Pct(), 1); Serial.print("%)  ");
    Serial.print("TPS2: "); Serial.print(GetTPS2Volts(), 3); Serial.print("V ("); Serial.print(GetTPS2Pct(), 1); Serial.println("%)");
    if (GetThrottleOpenLoopActive()) {
      const char *dir = GetThrottleOpenLoopDir() > 0 ? "FWD" : (GetThrottleOpenLoopDir() < 0 ? "REV" : "--");
      Serial.print("Throttle OPEN-LOOP: ");
      Serial.print(dir);
      Serial.print(' ');
      Serial.print(GetThrottleOpenLoopDuty());
      Serial.print("%  position: ");
      Serial.print(GetThrottlePosition(), 1);
      Serial.print("%  current: ");
      Serial.print(GetThrottleCurrentMa());
      Serial.println("mA");
    } else {
      Serial.print("Throttle target: "); Serial.print(GetThrottleTargetPct(), 1);
      Serial.print("%  position: "); Serial.print(GetThrottlePosition(), 1);
      Serial.print("%  current: "); Serial.print(GetThrottleCurrentMa());
      Serial.print("mA  fault: "); Serial.println(GetThrottleFault());
    }
  }
}
