// tasks.cpp
#include "tasks.h"

void MngTASK_Init(void){
    // We add the task to the task scheduler
    runner.addTask(Mng10ms);
    runner.addTask(Mng100ms);
    runner.addTask(Mng1000ms);

    runner.addTask(DelayShift);
    runner.addTask(DelayEngage);

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
}
void MngTASK_100ms(void){
    MngSHFT_100ms();
}
void MngTASK_1000ms(void){
    MngSHFT_1000ms();
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