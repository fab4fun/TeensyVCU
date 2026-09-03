// ThrottleCtrl.cpp
// eCharger boost bypass valve throttle-body position controller.
// Reads dual TPS feedback (via BoostSensor), drives the HB1/HB2 motor
// H-bridge under PID closed-loop control, target position from CAN.
//
// H-bridge topology (confirmed): each of HB1/HB2 is its own IR2104
// half-bridge driver channel - IN sets that leg's output (PWM duty
// controls the average node voltage), ~SD (active-low) shuts the whole
// channel down. Motor+ = HB1 leg (IN=DO2, ~SD=DO6), Motor- = HB2 leg
// (IN=DO3, ~SD=DO14). A full H-bridge is formed by the two independent
// half-bridges driving the motor's two terminals.
#include "ThrottleCtrl.h"
#include "BoostSensor.h"
#include "CAN.h"
#include <PID_v1.h>

// H-bridge topology confirmed 2026-08-31 (IR2104 driver chips, see note above).
#define THROTTLE_HBRIDGE_VERIFIED true

// !!! PLACEHOLDER CAN IDs !!!
// TODO: replace with the real MS3Pro TunerStudio CAN broadcast ID/layout
// for boost target before use. 0x300/0x301 are unused-but-unconfirmed
// placeholders (repo-wide grep this session only found 0x195/0x454/0x241
// in use), chosen only to avoid collisions with those.
const uint32_t THROTTLE_CAN_RX_ID_MS3PRO = 0x300; // TODO: confirm real ID/layout with MS3Pro CAN broadcast config
const uint32_t THROTTLE_CAN_RX_ID_DIRECT = 0x301; // secondary generic/direct override, for future use
const uint32_t THROTTLE_CAN_TX_ID_STATUS = 0x242; // status TX, next unused ID after shift's 0x241

// Motor drive pins (Configurations.md: HB1=DO2/6, HB2=DO3/14 - IN/~SD pairs per IR2104 channel)
const int motorHB1_IN_Pin = 2;
const int motorHB1_SD_Pin = 6;
const int motorHB2_IN_Pin = 3;
const int motorHB2_SD_Pin = 14;

// Current sense pin (Configurations.md CURR1 = AN10, confirmed)
const int currSensePin = A10;
const int analogMax = 1023;
int throttleCurrSenseRaw = 0;
long throttleCurrSenseVolt = 0; // mV relative to zero-current offset, signed (near-zero current reads below the offset)
const int currSenseZero = 1651;  // ADC zero-point (mV), ACS725-20AB; bench-adjusted +11mV (was reading 160mA at zero current with 1640)
// PLACEHOLDER: bench testing showed this reading ~2x the power supply's actual current, so halved
// from 15 as an interim correction. Needs an exact bench pair (PSU mA vs Serial-reported mA at the
// same instant) to compute the real gain precisely, same approach as the TPS calibration.
const float currSenseGain = 4.0f;   // mA/mV, ACS725-20AB
uint throttleCurrSense = 0;              // mA
const uint currLim = 2000;       // mA fault threshold
const int currLimDelay = 200;    // x10ms cycles until fault
int throttleCurrLimCount = 0;

// TPS calibration (bench-measured 2026-08-31, revised same day). TPS1 has a closed-end
// deadband - it reads a near-constant ~3.97-3.98V for roughly the first 6% of travel - so
// it can't reliably resolve position there. TPS2 is used as the PRIMARY reference channel
// instead (simple 2-point linear, rising as valve opens). TPS1 is only used as a cross-check:
// its expected voltage at the current TPS2-derived position is looked up from a piecewise
// table (bench points) and compared against its actual voltage.
#define TPS_CALIBRATION_MODE 0
const float tps2ClosedVolts = 0.494f;
const float tps2WideOpenVolts = 3.910f;

// TPS1 cross-check table: TPS2-based percent (ascending) -> expected TPS1 volts.
// The flat 0-6.1% segment captures TPS1's known closed-end deadband.
const int tps1CalPoints = 5;
const float tps1CalPct[tps1CalPoints]   = {0.0f,   6.1f,   50.4f,  65.5f, 100.0f};
const float tps1CalVolts[tps1CalPoints] = {3.983f, 3.974f, 2.703f, 2.22f, 0.396f};
const float tps1MismatchTolVolts = 0.4f; // bench-tunable; covers deadband + sensor noise

float tpsPosition1Pct = 0.0f;      // informational only - see deadband note above
float tpsPosition2Pct = 0.0f;      // == tpsPositionPct, kept for debug/status symmetry
float tpsPositionPct = 0.0f;       // primary position (TPS2-derived) used for control
float lastTps1Volts = 0.0f;
float lastTps2Volts = 0.0f;

// TPS1/TPS2 cross-check fault — windowed scheme. Motor commutation noise produces occasional
// single-sample glitches in the TPS reading, so a lone bad sample must not trip the fault.
// Keep a rolling window of the last `tpsMismatchWindow` samples (each 0=ok / 1=mismatch);
// >= `tpsMismatchFailThreshold` failures within the window trips the diagnostic, and it only
// clears once the entire window is clean again (all good) - giving clean hysteresis.
const int tpsMismatchWindow = 20;      // rolling measurement window (x10ms samples)
const int tpsMismatchFailThreshold = 16; // >= this many failures in-window trips the fault
int tpsMismatchBuf[tpsMismatchWindow];  // ring buffer: 1=mismatch, 0=ok (zero-initialized)
int tpsMismatchIdx = 0;                 // next slot to write
boolean tpsFault = false;

void UpdateTpsMismatchDiag(boolean mismatchNow) {
  tpsMismatchBuf[tpsMismatchIdx] = mismatchNow ? 1 : 0;
  tpsMismatchIdx = (tpsMismatchIdx + 1) % tpsMismatchWindow;

  int failCount = 0;
  for (int i = 0; i < tpsMismatchWindow; i++) failCount += tpsMismatchBuf[i];

  if (!tpsFault && failCount >= tpsMismatchFailThreshold) {
    tpsFault = true; // trip: 8/10 failing in the window
  } else if (tpsFault && failCount == 0) {
    tpsFault = false; // clear only when all samples in the window are good again
  }
}

// Stall/timeout fault
const int stallTimeoutDelay = 300; // x10ms cycles (3s) of no progress at high effort
const float stallProgressTolPct = 1.0f;
int stallCount = 0;
float lastStallCheckPct = 0.0f;
boolean stallFault = false;

boolean throttleFault = false;

// PID
double pidInput = 0.0;
double pidOutput = 0.0;
double pidSetpoint = 0.0;
double pidKp = 0.8, pidKi = 2.0, pidKd = 0.0; // feedforward carries the spring hold-load, so only small
// gains are needed to trim residual position error - keep them modest (large Ki reintroduces windup).
// DIRECT: with the corrected wiring (HB1 on Motor+, HB2 on Motor-), SetMotorDrive(+1) drives
// +voltage to Motor+ which OPENS the throttle valve against its spring - i.e. +output leads to
// +input, a direct-acting process. error = setpoint - input > 0 (need to open more) yields a
// positive output -> SetMotorDrive(+1) -> opens: negative feedback. NOTE: this used to be REVERSE
// as a workaround for the original (reversed) wiring; that workaround is now wrong and would
// reintroduce positive feedback (drives to one hard stop regardless of target).
PID throttlePID(&pidInput, &pidOutput, &pidSetpoint, pidKp, pidKi, pidKd, DIRECT);
const float positionDeadbandPct = 1.0f;

// Feedforward hold-load map: commanded position% -> base FWD duty% needed to HOLD that position against
// the return spring (the load pure-P cannot carry - which is why Kp-only settled ~77% at a 100% command).
// The PID above only trims residual error on top of this.
//
// Feedforward hold-load map: commanded position% -> base drive duty% needed to HOLD that position against
// the spring forces. The PID above only trims residual error on top of this.
//
// Mechanical model (bench-confirmed):
//   - Main long-wrap torsion spring: zero net force at ~5% position, constant opposing force above 5%.
//     → Positive FWD duty needed to hold open (>5%), roughly flat through the working range (~10-14%).
//   - Secondary seat/stop spring below 5%: pushes valve toward OPEN (away from fully-closed stop).
//     → Negative (REV) duty needed to hold in the 0-5% region.
//   - At ~5%: both springs balanced, zero net drive needed.
//
// The control code handles sign naturally: totalDrive > +deadband → FWD; < −deadband → REV.
// Tune by open-loop sweep: for each target position, note the smallest steady duty that holds with no drift.
const int ffHoldPoints = 5;
const float ffHoldPct[ffHoldPoints]   = {0.0f,    5.0f, 10.0f,   50.0f,  100.0f};
const float ffHoldDuty[ffHoldPoints]  = {-12.0f,   0.0f, 5.0f,  10.0f,  12.0f}; // signed: +FWD / −REV duty% (bench-tune)

// Open-loop bench-test mode: when active, MngThrottleCtrl_10ms() skips the PID loop and all fault
// latching - just holds a fixed direction/duty via SetMotorDrive(), so the raw H-bridge can be
// verified (consistent drive, correct direction) without any closed-loop interference.
boolean throttleTestMode = false;
int8_t  throttleTestDir = 0;
uint8_t throttleTestDuty = 0;

CAN_message_t throttleStatusMsg;

void EnableMotorDriver(boolean enable) {
  // ~SD is active-low: HIGH = channel enabled, LOW = hard shutdown (both outputs off)
  digitalWrite(motorHB1_SD_Pin, enable ? HIGH : LOW);
  digitalWrite(motorHB2_SD_Pin, enable ? HIGH : LOW);
}

void SetMotorDrive(int8_t direction, uint8_t dutyPercent) {
#if THROTTLE_HBRIDGE_VERIFIED
  int duty = map(dutyPercent, 0, 100, 0, 255);
  EnableMotorDriver(true);
  if (direction > 0) {
    analogWrite(motorHB2_IN_Pin, 0);
    analogWrite(motorHB1_IN_Pin, duty);
  } else if (direction < 0) {
    analogWrite(motorHB1_IN_Pin, 0);
    analogWrite(motorHB2_IN_Pin, duty);
  } else {
    // both legs LOW = both low-side FETs on = dynamic braking
    analogWrite(motorHB1_IN_Pin, 0);
    analogWrite(motorHB2_IN_Pin, 0);
  }
#else
  // Motor drive disabled: H-bridge topology not yet verified on the bench.
  EnableMotorDriver(false);
  digitalWrite(motorHB1_IN_Pin, LOW);
  digitalWrite(motorHB2_IN_Pin, LOW);
#endif
}

void MngThrottleCtrl_Init() {
  pinMode(motorHB1_IN_Pin, OUTPUT);
  pinMode(motorHB2_IN_Pin, OUTPUT);
  pinMode(motorHB1_SD_Pin, OUTPUT);
  pinMode(motorHB2_SD_Pin, OUTPUT);
  analogWriteFrequency(motorHB1_IN_Pin, 20000);
  analogWriteFrequency(motorHB2_IN_Pin, 20000);
  SetMotorDrive(0, 0);

  throttleStatusMsg.id = THROTTLE_CAN_TX_ID_STATUS;
  throttleStatusMsg.len = 8;

  throttlePID.SetMode(AUTOMATIC);
  throttlePID.SetOutputLimits(-100, 100);
}

float InterpolatePiecewise(float x, const float *xs, const float *ys, int n) {
  if (x <= xs[0]) return ys[0];
  if (x >= xs[n - 1]) return ys[n - 1];
  for (int i = 0; i < n - 1; i++) {
    if (x <= xs[i + 1]) {
      float t = (x - xs[i]) / (xs[i + 1] - xs[i]);
      return ys[i] + t * (ys[i + 1] - ys[i]);
    }
  }
  return ys[n - 1];
}

void ReadThrottlePosition() {
  BoostSensorData data = MngBoostSensor_Read();
  lastTps1Volts = data.throttlePos1;
  lastTps2Volts = data.throttlePos2;

  tpsPositionPct = ((data.throttlePos2 - tps2ClosedVolts) / (tps2WideOpenVolts - tps2ClosedVolts)) * 100.0f;
  tpsPosition2Pct = tpsPositionPct;
  tpsPosition1Pct = tpsPositionPct; // TPS1 can't independently resolve position near closed (deadband)

  float expectedTps1Volts = InterpolatePiecewise(tpsPositionPct, tps1CalPct, tps1CalVolts, tps1CalPoints);

#if TPS_CALIBRATION_MODE
  static uint32_t lastPrintMs = 0;
  if (millis() - lastPrintMs > 500) {
    lastPrintMs = millis();
    Serial.printf("TPS CAL: TPS1=%.3fV (expected %.3fV)  TPS2=%.3fV (%.1f%%)\n",
      data.throttlePos1, expectedTps1Volts, data.throttlePos2, tpsPositionPct);
  }
#endif

  // Windowed cross-check: a single bad sample just records one failure in the ring buffer;
  // the fault only trips at >=8/10 and clears when the whole window is clean again.
  UpdateTpsMismatchDiag(fabs(data.throttlePos1 - expectedTps1Volts) > tps1MismatchTolVolts);
}

void ReadMotorCurrent() {
  throttleCurrSenseRaw = analogRead(currSensePin);
  throttleCurrSenseVolt = ((long)throttleCurrSenseRaw * 3300) / analogMax - currSenseZero;
  // magnitude only - ACS725 is bidirectional around the zero offset, fault threshold cares about |current|
  throttleCurrSense = (uint)fabs((float)throttleCurrSenseVolt * currSenseGain);

  if (throttleCurrSense > currLim) {
    throttleCurrLimCount++;
  } else {
    throttleCurrLimCount = 0;
  }
}

void CheckStall() {
  if (fabs(pidOutput) > 80.0) { // near-max effort commanded
    if (fabs(tpsPositionPct - lastStallCheckPct) < stallProgressTolPct) {
      stallCount++;
    } else {
      stallCount = 0;
      lastStallCheckPct = tpsPositionPct;
    }
  } else {
    stallCount = 0;
    lastStallCheckPct = tpsPositionPct;
  }
  stallFault = (stallCount > stallTimeoutDelay);
}

void MngThrottleCtrl_10ms(void) {
  ReadThrottlePosition();
  ReadMotorCurrent();

  if (throttleTestMode) {
    // Open-loop bench-test mode: hold a fixed direction/duty, skip PID + all fault latching.
    SetMotorDrive(throttleTestDir, throttleTestDuty);
    return;
  }

  throttleFault = tpsFault || stallFault || (throttleCurrLimCount > currLimDelay);

  if (throttleFault) {
    // Capture causes before resetting counters below (reset would otherwise hide them from the print).
    boolean wasCurrLimFault = (throttleCurrLimCount > currLimDelay);
    boolean wasStallFault = stallFault;

    // Hard-disable both IR2104 channels (~SD LOW) rather than dynamic-braking via SetMotorDrive(0,0) -
    // removes gate drive entirely instead of leaving both low-side FETs conducting.
    EnableMotorDriver(false);
    digitalWrite(motorHB1_IN_Pin, LOW);
    digitalWrite(motorHB2_IN_Pin, LOW);
    // Zero the stall tracker so a latched stall fault can clear once the triggering fault (e.g. TPS mismatch) resolves,
    // instead of CheckStall() never running again and leaving stallCount/pidOutput frozen above threshold forever.
    // Also reset the PID's internal integral accumulator (not just our external pidOutput) - otherwise it stays
    // saturated from whatever windup caused the stall, and immediately re-saturates/re-stalls a few seconds after
    // the fault clears instead of actually recovering (toggling MANUAL/AUTOMATIC is PID_v1's documented reset idiom).
    pidOutput = 0.0;
    throttlePID.SetMode(MANUAL);
    throttlePID.SetMode(AUTOMATIC);
    stallCount = 0;
    lastStallCheckPct = tpsPositionPct;
    stallFault = false;
    throttleCurrLimCount = 0;

    static uint32_t lastFaultPrintMs = 0;
    if (millis() - lastFaultPrintMs > 500) {
      lastFaultPrintMs = millis();
      if (wasCurrLimFault) Serial.println("ThrottleCtrl: overcurrent fault, motor disabled");
      if (tpsFault) Serial.println("ThrottleCtrl: TPS1/TPS2 mismatch fault, motor disabled");
      if (wasStallFault) Serial.println("ThrottleCtrl: stall/timeout fault, motor disabled");
    }
    return;
  }

  pidInput = tpsPositionPct;
  throttlePID.Compute();

  CheckStall();

  // Feedforward + PID: the feedforward table carries the spring hold-load at the commanded position.
  // The PID output (small, ±) trims residual error on top of that base duty.
  //   total > 0 → FWD direction at |total|% duty (HB1 PWM on Motor+)
  //   total < 0 → REV direction at |total|% duty (HB2 PWM on Motor−)
  //   total ≈ 0 → both off (dynamic brake; only when target is near fully-closed and PID ~0)
  float ffDuty = InterpolatePiecewise((float)pidSetpoint, ffHoldPct, ffHoldDuty, ffHoldPoints);
  double totalDrive = ffDuty + pidOutput;

  if (totalDrive > positionDeadbandPct) {
    SetMotorDrive(1, (uint8_t)constrain(totalDrive, 0.0, 100.0));
  } else if (totalDrive < -positionDeadbandPct) {
    SetMotorDrive(-1, (uint8_t)constrain(-totalDrive, 0.0, 100.0));
  } else {
    // Within deadband of zero net drive (target near fully-closed + PID ~0): hold off.
    SetMotorDrive(0, 0);
  }
}

void BuildThrottleCtrl_StatusMsg() {
  throttleStatusMsg.buf[0] = (uint8_t)constrain(tpsPositionPct, 0, 255);
  throttleStatusMsg.buf[1] = (uint8_t)constrain(pidSetpoint, 0, 255);
  throttleStatusMsg.buf[2] = throttleCurrSense & 0xFF;
  throttleStatusMsg.buf[3] = (throttleCurrSense >> 8) & 0xFF;
  throttleStatusMsg.buf[4] = throttleFault ? 1 : 0;
  throttleStatusMsg.buf[5] = tpsFault ? 1 : 0;
  throttleStatusMsg.buf[6] = stallFault ? 1 : 0;
  throttleStatusMsg.buf[7] = 0; // reserved
  CAN_Send(throttleStatusMsg);
}

void MngThrottleCtrl_100ms(void) {
  BuildThrottleCtrl_StatusMsg();
}

void CAN_Parse_Throttle(const CAN_message_t &msg) {
  if (msg.id == THROTTLE_CAN_RX_ID_MS3PRO) {
    // TODO: confirm real byte layout once MS3Pro CAN broadcast config is known.
    pidSetpoint = msg.buf[0];
  }
  if (msg.id == THROTTLE_CAN_RX_ID_DIRECT) {
    pidSetpoint = msg.buf[0];
  }
}

float GetThrottlePosition(void) {
  return tpsPositionPct;
}

boolean GetThrottleFault(void) {
  return throttleFault;
}

float GetTPS1Volts(void) {
  return lastTps1Volts;
}

float GetTPS2Volts(void) {
  return lastTps2Volts;
}

float GetTPS1Pct(void) {
  return tpsPosition1Pct;
}

float GetTPS2Pct(void) {
  return tpsPosition2Pct;
}

float GetThrottleTargetPct(void) {
  return (float)pidSetpoint;
}

uint GetThrottleCurrentMa(void) {
  return throttleCurrSense;
}

void SetThrottleTargetPct(float pct) {
  pidSetpoint = constrain(pct, 0.0f, 100.0f);
}

void SetThrottleOpenLoop(int8_t direction, uint8_t dutyPercent) {
  throttleTestDir = (int8_t)constrain(direction, -1, 1);
  throttleTestDuty = constrain(dutyPercent, 0, 100);
  throttleTestMode = true;
}

void ExitThrottleOpenLoop(void) {
  throttleTestMode = false;
  SetMotorDrive(0, 0); // hold off (dynamic braking) so closed-loop PID takes over cleanly
  // Reset the PID integral accumulator so it doesn't carry stale windup from before test mode.
  pidOutput = 0.0;
  throttlePID.SetMode(MANUAL);
  throttlePID.SetMode(AUTOMATIC);
}

boolean GetThrottleOpenLoopActive(void) { return throttleTestMode; }
int8_t  GetThrottleOpenLoopDir(void)    { return throttleTestDir; }
uint8_t GetThrottleOpenLoopDuty(void)   { return throttleTestDuty; }
