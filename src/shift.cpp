// shift.cpp
#include "shift.h"
#include "CAN.h" // Add this include

//#define _TASK_SLEEP_ON_IDLE_RUN


extern void MngTASK_ShiftTimer(int);
extern void MngTASK_EngageTimer(int);
extern void MngTASK_ShiftDisable(void);
extern void MngTASK_EngageDisable(void);

// CAN configuration
//FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> PriCAN;
//#define NUM_TX_MAILBOXES 2
//#define NUM_RX_MAILBOXES 2

CAN_message_t inMsg;
CAN_message_t outMsg;
CAN_message_t initMsg;
CAN_message_t KEY_RxMsg;
CAN_message_t LED_TxMsg;
CAN_message_t ShiftServo_1_TxMsg;

/*boolean CAN_SW1;  //shift up
uint8_t CAN_SW1_byte = 0;
uint8_t CAN_SW1_mask = 0x02;
boolean CAN_SW1_pend;
boolean CAN_SW2;  //shift down
uint8_t CAN_SW2_byte = 0;
uint8_t CAN_SW2_mask = 0x80;
boolean CAN_SW2_pend;
boolean CAN_SW3; // neutral
uint8_t CAN_SW3_byte = 0;
uint8_t CAN_SW3_mask = 0x04;
boolean CAN_SW3_pend;
boolean CAN_SW4; // third
uint8_t CAN_SW4_byte = 1;
uint8_t CAN_SW4_mask = 0x01;
boolean CAN_SW4_pend;*/

//uint8_t CAN_SW_byte[12] = {0,0,0,0,0,0,0,0,1,1,1,1};
//uint8_t CAN_SW_mask[12] = {0x01,0x02,0x04,,0,0,0,0,1,1,1,1}
uint16_t CAN_SW_read;
//boolean CAN_SW[12];
//boolean CAN_SW_pend[12];

// 12 maps to no switch
// index: 0 1 2 3 4 5 6 7  8
// gear:  R N 1 2 3 4 5 UP DOWN
//
// ExhOn UP 1 2 3 4
// ExhOff DN N x x x
uint8_t CAN_SW_Map[9] = {12, 8, 2, 3, 4, 5, 12, 1, 7};


int16_t GearRatio[7] = {-1232, 0, 1342, 754, 502, 354, 276};  // [ratio*100]
int16_t currentRatio = 0;
int16_t TireCirc = 1875;   // [mm]
int16_t VehVel = 0;  // [km/h*100]

//PWMServo servo1, servo2;
Servo servoGate, servoEngage, servoClutch;

const int shiftUpPin = 9;
const int shiftDnPin = 10;
const int neutralPin = 11;
const int powerPin = 17;
const int servoPowerPin = 4;  // Halfbridge 1 HS only used for power to servos. Aslo needs DO6 set to <xxx>
const int servoPowerHB_Pin = 15;  // Halfbridge Set to TRUE for HS
#ifdef ARDUINO_TEENSY41
  const int currSensePin = A12;   // pin 14 (J3)
#else
  const int currSensePin = A6;   // jumper from current sensor output to A6
#endif

const int debounceDelay = 200;

const int servoGatePin = 18;
const int servoEngagePin = 19;
const int servoClutchPin = 28;

int currentGear = 0;   //  -1 reverse, 0 neutral, 1 - 5  
int targetGear = 0;
const int maxGears = 6;
int minGear = -1;
int neutralIndex = 1;

int servoGatePos[] = {24, 0, 12, 12, 0, 0, -12, 0};
int servoEngagePos[] = {25, 0, 25, -23, 25, -23, 25, 0};
int servoGateOffset = 102;
int servoEngageOffset = 85;

int shiftGateDelay = 200;
int shiftEngageDelay = 450;

int analogMax = 1023;
int currSenseRaw = 0;
u_long currSenseVolt = 0;
int currSenseZero = 1640; // (mv)
//int currSenseGain = 44;  //  (mv/A)   66mv/A at 5V converted to 3.3V
uint currSenseGain = 15;  //  15.15 (mA/mv)   66mv/A at 3.3V (ACS725-20AB)
uint currSense; // mA
uint currLim = 2000;  // mA fault threshold
int currLimDelay = 200;   // ms (x10) until fault 
int currLimCount = 0;

long motorVel = 0;  // (rpm)
boolean motorVelRcvd = 0; 

  boolean shiftUpPinSt = false;
  boolean shiftDnPinSt = false;
  boolean neutralPinSt = false;
  boolean powerPinSt = false;  

  boolean shiftUpPinPrevSt = false;
  boolean shiftDnPinPrevSt = false;
  boolean neutralPinPrevSt = false;
  boolean powerPinPrevSt = false;  

  int shiftUpPinDebncCnt = 0;
  int shiftDnPinDebncCnt = 0;
  int neutralPinDebncCnt = 0;
  int powerPinDebncCnt = 0;  

  int inputPinDebncSample = 10;

  boolean shiftUpCAN_St = false;
  boolean shiftDnCAN_St = false;
  boolean neutralCAN_St = false;
  boolean shift1stCAN_St = false;
  boolean shift2ndCAN_St = false;
  boolean shift3rdCAN_St = false;
  boolean shift4thCAN_St = false;
  boolean shift5thCAN_St = false;

  boolean shiftUpCAN_PrevSt = false;
  boolean shiftDnCAN_PrevSt = false;
  boolean neutralCAN_PrevSt = false;
  boolean shift1stCAN_PrevSt = false;
  boolean shift2ndCAN_PrevSt = false;
  boolean shift3rdCAN_PrevSt = false;
  boolean shift4thCAN_PrevSt = false;
  boolean shift5thCAN_PrevSt = false;

void MngSHFT_Init() {
  // put your setup code here, to run once:
  //Serial.begin(9600); // USB is always 12 or 480 Mbit/sec



  //test message
  outMsg.id = 0x241;
  outMsg.len = 8;
  //outMsg.buf[0] = 0xEE;

  pinMode(servoGatePin,OUTPUT);
  pinMode(servoEngagePin,OUTPUT);
  pinMode(servoClutchPin,OUTPUT);

  servoGate.attach(servoGatePin);
  // servoGate.write(0);

  servoEngage.attach(servoEngagePin);
  servoClutch.attach(servoClutchPin);
  servoClutch.write(servoEngagePos[neutralIndex]+servoEngageOffset);
  disengage();

  //for pins borrowed from 3.2 setup
  //pinMode(3,INPUT_DISABLE);
  //pinMode(4,INPUT_DISABLE);
  //pinMode(20,INPUT);

  pinMode(shiftUpPin, INPUT);
  pinMode(shiftDnPin, INPUT);
  pinMode(neutralPin, INPUT);
  pinMode(powerPin, INPUT);

  pinMode(servoPowerPin,OUTPUT);
  pinMode(servoPowerHB_Pin,OUTPUT);
  pinMode(5,OUTPUT);
  // enable power to servos
   digitalWrite(servoPowerPin,LOW);
    digitalWrite(5,HIGH);
    digitalWrite(servoPowerHB_Pin,HIGH);
 // analogWriteFrequency(5, 1);
 // analogWrite(5, 127); // 99% duty cycle

}

void MngSHFT_loop() {

}

void MngSHFT_10ms() {
  shiftUpPinSt = digitalRead(shiftUpPin);
  shiftDnPinSt = digitalRead(shiftDnPin);
  neutralPinSt = digitalRead(neutralPin);
  powerPinSt = digitalRead(powerPin);  

  currSenseRaw = analogRead(currSensePin);
  currSenseVolt = ((currSenseRaw * 3300) / analogMax) - currSenseZero;
  currSense = currSenseVolt * currSenseGain;

 if(currSense > currLim) {
  currLimCount++;
 }
 else {
  currLimCount=0;
 }
 if(currLimCount>currLimDelay) { // shift to neutral and reset all requests
    disengage();
    Serial.println("Overcurrent, disengage to neutral");
    currentGear = neutralIndex-1;
    targetGear = currentGear;
    currLimCount=0;
    neutralPinSt = false;
    shiftUpPinSt = false;
    shiftDnPinSt = false;
    neutralCAN_St = false; // reset all latched requests
    shiftUpCAN_St = false;
    shiftDnCAN_St = false;
    shift3rdCAN_St = false;
}
else {

  if (digitalRead(shiftUpPin) != shiftUpPinPrevSt) {
    shiftUpPinDebncCnt++;
    if (shiftUpPinDebncCnt == inputPinDebncSample) {
      shiftUpPinSt = !shiftUpPinPrevSt;
      shiftUpPinDebncCnt = 0;
      shiftUpPinPrevSt = shiftUpPinSt;
    }
  }
  else {
    shiftUpPinDebncCnt = 0;
  }

  if (digitalRead(shiftDnPin) != shiftDnPinPrevSt) {
    shiftDnPinDebncCnt++;
    if (shiftDnPinDebncCnt == inputPinDebncSample) {
      shiftDnPinSt = !shiftDnPinPrevSt;
      shiftDnPinDebncCnt = 0;
      shiftDnPinPrevSt = shiftDnPinSt;
    }
  }
  else {
    shiftDnPinDebncCnt = 0;
  }

  if (digitalRead(neutralPin) != neutralPinPrevSt) {
    neutralPinDebncCnt++;
    if (neutralPinDebncCnt == inputPinDebncSample) {
      neutralPinSt = !neutralPinPrevSt;
      neutralPinDebncCnt = 0;
      neutralPinPrevSt = neutralPinSt;
    }
  }
  else {
    neutralPinDebncCnt = 0;
  }

  if (neutralPinSt || neutralCAN_St){

      targetGear = neutralIndex-1;
      Serial.println("Neutral");
      disengage();
      MngTASK_ShiftTimer(shiftGateDelay);
      //DelayEngage.enableDelayed(shiftEngageDelay);
      //shift();
      currentGear = targetGear;  // normally set after DelayEngage
      neutralPinSt = false;
      shiftUpPinSt = false;
      shiftDnPinSt = false;
      neutralCAN_St = false; // reset all latched requests
      shiftUpCAN_St = false;
      shiftDnCAN_St = false;
      shift1stCAN_St = false;
      shift2ndCAN_St = false;
      shift3rdCAN_St = false;
      shift4thCAN_St = false;
      shift5thCAN_St = false;
    }

  if(currentGear == targetGear){
    if(shiftUpPinSt || shiftUpCAN_St) {   

      if(currentGear < maxGears) {
        targetGear = currentGear+1;
        Serial.println("Shift Up");
        Serial.println(targetGear);
        disengage();
        MngTASK_ShiftTimer(shiftGateDelay);
        MngTASK_EngageTimer(shiftEngageDelay);
        //shift();
        //engage();
      }
      else {
        Serial.println("Denied: at max gear");
      }

      shiftUpPinSt = false;
      shiftUpCAN_St = false;  // reset latched request
    }
    else if (shiftDnPinSt || shiftDnCAN_St) {
      if(currentGear > minGear) {
        if(currentGear ==0 && (!motorVelRcvd || (motorVel > 10)) ){
          Serial.println("Denied: Motor speed too high");
        }
        else if(currentGear >=2 && (!motorVelRcvd || (motorVel > 5000))){
          Serial.println("Denied: Motor speed too high");
        }
        else {
          targetGear = currentGear-1;
          Serial.println("Shift Down");
          Serial.println(targetGear);
          disengage();
          MngTASK_ShiftTimer(shiftGateDelay);
          MngTASK_EngageTimer(shiftEngageDelay);
        }  
      }
      else {
        Serial.println("Denied: at min gear");
      }     
      shiftDnPinSt = false;
      shiftDnCAN_St = false;  // reset latched request

    }
    else if (shift4thCAN_St) {
      targetGear = 4;
      Serial.println("Shift to 4th");
      disengage();
      MngTASK_ShiftTimer(shiftGateDelay);
      MngTASK_EngageTimer(shiftEngageDelay);
      shift4thCAN_St = false;  // reset latched request
    }
    else if (shift3rdCAN_St) {
      targetGear = 3;
      Serial.println("Shift to 3rd");
      disengage();
      MngTASK_ShiftTimer(shiftGateDelay);
      MngTASK_EngageTimer(shiftEngageDelay);
      shift3rdCAN_St = false;  // reset latched request
    }
    else if (shift2ndCAN_St) {
      targetGear = 2;
      Serial.println("Shift to 2nd");
      disengage();
      MngTASK_ShiftTimer(shiftGateDelay);
      MngTASK_EngageTimer(shiftEngageDelay);
      shift2ndCAN_St = false;  // reset latched request
    }
    else if (shift1stCAN_St) {
      if((currentGear<=0) && (!motorVelRcvd || (motorVel > 10)) ){
          Serial.println("Denied: Motor speed too high");
      }
      else if((currentGear==2) && (!motorVelRcvd || (motorVel > 5000))){
          Serial.println("Denied: Motor speed too high");
      }
      else if((currentGear >=3) && (!motorVelRcvd || (motorVel > 2000))){
          Serial.println("Denied: Motor speed too high");
      }
      else {
        targetGear = 1;
        Serial.println("Shift to 1st");
        disengage();
        MngTASK_ShiftTimer(shiftGateDelay);
        MngTASK_EngageTimer(shiftEngageDelay);

      }
        shift1stCAN_St = false;  // reset latched request
    }
  }

}

}

void MngSHFT_100ms() {
  DtrmnSHFT_RatioVelEst();
  BuildSHFT_GearStatusMsg();
}

void DtrmnSHFT_RatioVelEst()
{
  currentRatio = GearRatio[(currentGear + 1)];
  if (currentGear != 0)
  {                                                             // handle neutral div/0 case
    VehVel = ((motorVel * 60 * TireCirc) / currentRatio) / 100; // km/h*100 = 100 * RPM * (mm) / (ratio*100)/100 / (hr/min * mm/km)
  }
}

void BuildSHFT_GearStatusMsg()
{
  outMsg.buf[0] = currentGear;                   // R(-1) N(0) 1 2 3 4 5
  outMsg.buf[1] = (abs(currSense) / 100) & 0xFF; // A * 10
  outMsg.buf[2] = ((uint16_t)VehVel >> 0) & 0xFF;
  outMsg.buf[3] = ((uint16_t)VehVel >> 8) & 0xFF;
  outMsg.buf[4] = ((uint16_t)currentRatio >> 0) & 0xFF; // ratio *100
  outMsg.buf[5] = ((uint16_t)currentRatio >> 8) & 0x0F;
  outMsg.buf[6] = targetGear; // R(-1) N(0) 1 2 3 4 5
  CAN_Send(outMsg);           // Replace PriCAN.write(outMsg) with this call
}

void MngSHFT_1000ms() {
 // Serial.println("Tic");
  Serial.print("Current Sensor (mA): ");
  Serial.print(currSense);
  Serial.print(" Vehicle speed (km/h): ");
  Serial.print(VehVel/100);
  Serial.print(" Motor (RPM): ");
  Serial.println(motorVel,DEC);
  // PriCAN.mailboxStatus();
}

void shift() {
    Serial.println("Shift Task");
    servoGate.write(servoGatePos[targetGear+1]+servoGateOffset);
    MngTASK_ShiftDisable();
}

void engage() {
    Serial.println("Engage Task");
    servoEngage.write(servoEngagePos[targetGear+1]+servoEngageOffset);
    MngTASK_EngageDisable();
    currentGear = targetGear;
}

void disengage() {
    servoEngage.write(servoEngagePos[neutralIndex]+servoEngageOffset);
}

void CAN_Parse_Shift(const CAN_message_t &msg) {
/*  Serial.print("MB "); Serial.print(msg.mb);
  Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
  Serial.print("  LEN: "); Serial.print(msg.len);
  Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  Serial.print(" TS: "); Serial.print(msg.timestamp);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } Serial.println(); */

  if (msg.id == 0x195)
  {
      // if signal was off and now on, log a key press and toggle state of SW1
      // (only triggers once on a low to high transition)
      // only allow one command at a time.  priority to neutral
      CAN_SW_read = msg.buf[0]+(msg.buf[1]<<8);
      if (!neutralCAN_PrevSt && !neutralCAN_St && (CAN_SW_read&(1<<CAN_SW_Map[1]))) {
        neutralCAN_St = true;
        neutralCAN_PrevSt = true;
      }  
      else if (!shiftUpCAN_PrevSt && !shiftUpCAN_St && (CAN_SW_read&(1<<CAN_SW_Map[7]))) {
        shiftUpCAN_St = true;
        shiftUpCAN_PrevSt = true;
      }
      else if (!shiftDnCAN_PrevSt && !shiftDnCAN_St && (CAN_SW_read&(1<<CAN_SW_Map[8]))) {
        shiftDnCAN_St = true;
        shiftDnCAN_PrevSt = true;
      }
      else if (!shift1stCAN_PrevSt && !shift1stCAN_St && (CAN_SW_read&(1<<CAN_SW_Map[2]))) {
        shift1stCAN_St = true;
        shift1stCAN_PrevSt = true;
      }
      else if (!shift2ndCAN_PrevSt && !shift2ndCAN_St && (CAN_SW_read&(1<<CAN_SW_Map[3]))) {
        shift2ndCAN_St = true;
        shift2ndCAN_PrevSt = true;
      }
      else if (!shift3rdCAN_PrevSt && !shift3rdCAN_St && (CAN_SW_read&(1<<CAN_SW_Map[4]))) {
        shift3rdCAN_St = true;
        shift3rdCAN_PrevSt = true;
      }
      else if (!shift4thCAN_PrevSt && !shift4thCAN_St && (CAN_SW_read&(1<<CAN_SW_Map[5]))) {
        shift4thCAN_St = true;
        shift4thCAN_PrevSt = true;
      }
      else if (!shift5thCAN_PrevSt && !shift5thCAN_St && (CAN_SW_read&(1<<CAN_SW_Map[6]))) {
        shift5thCAN_St = true;
        shift5thCAN_PrevSt = true;
      }
      if (!(CAN_SW_read&(1<<CAN_SW_Map[7])))  // reset signal
      {
        shiftUpCAN_PrevSt = false;
      }
      if (!(CAN_SW_read&(1<<CAN_SW_Map[8])))  // reset signal
      {
        shiftDnCAN_PrevSt = false;
      }
      if (!(CAN_SW_read&(1<<CAN_SW_Map[1])))  // reset signal
      {
        neutralCAN_PrevSt = false;
      }
      if (!(CAN_SW_read&(1<<CAN_SW_Map[2])))  // reset signal
      {
        shift1stCAN_PrevSt = false;
      }
      if (!(CAN_SW_read&(1<<CAN_SW_Map[3])))  // reset signal
      {
        shift2ndCAN_PrevSt = false;
      }
      if (!(CAN_SW_read&(1<<CAN_SW_Map[4])))  // reset signal
      {
        shift3rdCAN_PrevSt = false;
      }
      if (!(CAN_SW_read&(1<<CAN_SW_Map[5])))  // reset signal
      {
        shift4thCAN_PrevSt = false;
      }
      if (!(CAN_SW_read&(1<<CAN_SW_Map[6])))  // reset signal
      {
        shift5thCAN_PrevSt = false;
      }
  }
  if(msg.id == 0x454) {

    motorVel = msg.buf[1] | msg.buf[2]<<8 | msg.buf[3]<<16 | msg.buf[4]<<24;
    motorVelRcvd = true;

  }
}