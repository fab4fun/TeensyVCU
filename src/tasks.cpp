// tasks.cpp
#include "tasks.h"

void MngTASK_Init(void){
    // We add the task to the task scheduler
    runner.addTask(Mng10ms);
    runner.addTask(Mng100ms);
    runner.addTask(Mng1000ms);

    runner.addTask(DelayShift);
    runner.addTask(DelayEngage);

    
    setSyncProvider(getTeensy3Time);  // will also sync time while setting up time source

    // We activate the task
    Mng10ms.enable();
    Mng100ms.enable();
    Mng1000ms.enable();
    MngSHFT_Init();


}

void MngTASK_Loop(void){
      // It is necessary to run the runner on each loop
  MngSHFT_loop();    
  runner.execute();
}

void MngTASK_10ms(void){
    MngSHFT_10ms();
    // testcode: reset power hold pin
    if(digitalRead(12) == HIGH){
        digitalWrite(13, HIGH);
    }
}
void MngTASK_100ms(void){
    MngSHFT_100ms();
}
void MngTASK_1000ms(void){
    MngSHFT_1000ms();
    if (GetGPS_b_Fix() && GetGPS_Cnt_Year() > 0 && GetGPS_t_SinceFix() < 3) {
        // set the Time to the latest GPS reading + time since reading
        setTime(GetGPS_t_Hour(), GetGPS_t_Minute(), GetGPS_t_Seconds()+(int)floor(GetGPS_t_SinceFix()), GetGPS_Cnt_Day(), GetGPS_Cnt_Month(), GetGPS_Cnt_Year());
        // adjust for GPS using UTC with hardcoded timezone
        adjustTime(timeZoneOffset * SECS_PER_HOUR);
        // update RTC clock
        Teensy3Clock.set(now());
      }
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
    digitalWrite(6, LOW);
    digitalWrite(14, LOW);
    digitalWrite(15, LOW);
    // testcode: delayed shutdown
    if(digitalRead(12) == LOW){
        digitalWrite(13, LOW);
    }

}

void MngTASK_ShiftTimer(int delay){
    DelayShift.enableDelayed(delay);
}
void MngTASK_EngageTimer(int delay){
    DelayEngage.enableDelayed(delay);
}
void MngTASK_ShiftDisable(void){
    DelayShift.disable();
}
void MngTASK_EngageDisable(void){
    DelayEngage.disable();
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}