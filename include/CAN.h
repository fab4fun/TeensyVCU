#ifndef CAN_H
#define CAN_H

#include <FlexCAN_T4.h>

// CAN configuration
extern FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> PriCAN;
#define NUM_TX_MAILBOXES 2
#define NUM_RX_MAILBOXES 2

extern CAN_message_t inMsg;
extern CAN_message_t outMsg;
extern CAN_message_t initMsg;
extern CAN_message_t KEY_RxMsg;
extern CAN_message_t LED_TxMsg;
extern CAN_message_t ShiftServo_1_TxMsg;

void MngCAN_Init();
void MngCAN_Loop();
void CAN_Send(const CAN_message_t& msg);
extern void CAN_Parse_Shift(const CAN_message_t &msg);

#endif // CAN_H