#ifndef TASKS_H
#define TASKS_H

#include <TaskScheduler.h>
//#include "shift.h"

void MngTASK_10ms(void);
void MngTASK_100ms(void);
void MngTASK_1000ms(void);

void MngTASK_Init(void);
void MngTASK_Loop(void);

void MngTASK_ShiftTimer(int);
void MngTASK_EngageTimer(int);
void MngTASK_ShiftDisable(void);
void MngTASK_EngageDisable(void);


extern void shift(void);
extern void engage(void);

// We create the Scheduler that will be in charge of managing the tasks
Scheduler runner;

Task Mng10ms(10, TASK_FOREVER, &MngTASK_10ms);
Task Mng100ms(100, TASK_FOREVER, &MngTASK_100ms);
Task Mng1000ms(1000, TASK_FOREVER, &MngTASK_1000ms);

Task DelayShift(TASK_IMMEDIATE, TASK_FOREVER, &shift);
Task DelayEngage(TASK_IMMEDIATE, TASK_FOREVER, &engage);

extern void MngSHFT_Init();
extern void MngSHFT_loop();

extern void MngSHFT_10ms(void);
extern void MngSHFT_100ms(void);
extern void MngSHFT_1000ms(void);

#endif // TASKS_H