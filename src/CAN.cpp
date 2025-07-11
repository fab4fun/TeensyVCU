#include "CAN.h"

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> PriCAN;


void MngCAN_Init() {
    PriCAN.begin();
    PriCAN.setBaudRate(500000L);
    PriCAN.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);

    for (int i = 0; i < NUM_RX_MAILBOXES; i++) {
        PriCAN.setMB((FLEXCAN_MAILBOX)i, RX, STD);
    }
    for (int i = NUM_RX_MAILBOXES; i < (NUM_TX_MAILBOXES + NUM_RX_MAILBOXES); i++) {
        PriCAN.setMB((FLEXCAN_MAILBOX)i, TX, STD);
    }

    PriCAN.setMBFilter(REJECT_ALL);
    PriCAN.enableMBInterrupts();
    PriCAN.onReceive(MB0, CAN_Parse_Shift);
    PriCAN.setMBFilter(MB0, 0x195);
    PriCAN.onReceive(MB1, CAN_Parse_Shift);
    PriCAN.setMBFilter(MB1, 0x454);
    PriCAN.mailboxStatus();
}

void MngCAN_Loop() {
    PriCAN.events();

 /* if ( PriCAN.read(inMsg) ) {
    Serial.print("CAN1 "); 
    Serial.print("MB: "); Serial.print(inMsg.mb);
    Serial.print("  ID: 0x"); Serial.print(inMsg.id, HEX );
    Serial.print("  EXT: "); Serial.print(inMsg.flags.extended );
    Serial.print("  LEN: "); Serial.print(inMsg.len);
    Serial.print(" DATA: ");
    for ( uint8_t i = 0; i < 8; i++ ) {
      Serial.print(inMsg.buf[i]); Serial.print(" ");
    }
    Serial.print("  TS: "); Serial.println(inMsg.timestamp);
  }*/

}

void CAN_Send(const CAN_message_t& msg) {
    PriCAN.write(msg);
}

// canSniff implementation should be moved here as well if not using shift.cpp variables