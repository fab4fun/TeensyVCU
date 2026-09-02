#ifndef THROTTLECTRL_H
#define THROTTLECTRL_H
#include <Arduino.h>
#include <FlexCAN_T4.h>

// TODO: placeholders - confirm real MS3Pro CAN broadcast ID/layout before use
extern const uint32_t THROTTLE_CAN_RX_ID_MS3PRO;
extern const uint32_t THROTTLE_CAN_RX_ID_DIRECT;
extern const uint32_t THROTTLE_CAN_TX_ID_STATUS;

void MngThrottleCtrl_Init();
void MngThrottleCtrl_10ms(void);
void MngThrottleCtrl_100ms(void);
void BuildThrottleCtrl_StatusMsg();
void CAN_Parse_Throttle(const CAN_message_t &msg);

float GetThrottlePosition(void);
boolean GetThrottleFault(void);
float GetTPS1Volts(void);
float GetTPS2Volts(void);
float GetTPS1Pct(void);
float GetTPS2Pct(void);
float GetThrottleTargetPct(void);
uint GetThrottleCurrentMa(void);
void SetThrottleTargetPct(float pct); // bench-test override, bypasses CAN target source

// Open-loop bench-test mode: drive the H-bridge with a fixed direction/duty, bypassing PID + fault logic.
void SetThrottleOpenLoop(int8_t direction, uint8_t dutyPercent); // +1/-1/0
void ExitThrottleOpenLoop(void);                                  // back to closed-loop PID
boolean GetThrottleOpenLoopActive(void);                          // true while in open-loop mode
int8_t  GetThrottleOpenLoopDir(void);                             // current direction (+1/-1/0)
uint8_t GetThrottleOpenLoopDuty(void);                            // current duty (0-100%)

#endif // THROTTLECTRL_H
