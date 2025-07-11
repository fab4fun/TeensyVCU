#ifndef SHIFT_H
#define SHIFT_H
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <servo.h>
//#include <TaskScheduler.h>
//#include "tasks.h"

void MngSHFT_Init();
void MngSHFT_loop();

void MngSHFT_10ms(void);
void MngSHFT_100ms(void);
void BuildSHFT_GearStatusMsg();
void DtrmnSHFT_RatioVelEst();
void MngSHFT_1000ms(void);

void shift(void);
void disengage(void);
void engage(void);
void CAN_Parse_Shift(const CAN_message_t &msg);

#endif // SHIFT_H