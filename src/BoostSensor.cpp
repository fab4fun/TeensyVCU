#include "BoostSensor.h"

// Pin assignments per docs/Configurations.md: AN1(A2)=TPS1, AN2(A3)=TPS2,
// X3_1(A14)=MAP (boost pressure), X3_3(A15)=IAT (boost air temp)
const uint8_t THROTTLE_POS1_PIN = A2;
const uint8_t THROTTLE_POS2_PIN = A3;
const uint8_t MAP_PIN          = A14;
const uint8_t INT_TEMP_PIN     = A15;

void MngBoostSensor_Init() {
    // Configure ADC pins as inputs (Arduino style)
    pinMode(THROTTLE_POS1_PIN, INPUT);
    pinMode(THROTTLE_POS2_PIN, INPUT);
    pinMode(MAP_PIN,           INPUT);
    pinMode(INT_TEMP_PIN,      INPUT);
}

BoostSensorData MngBoostSensor_Read() {
    BoostSensorData data;
    data.throttlePos1 = analogRead(THROTTLE_POS1_PIN) / 1023.0f * 5.0f; // example scaling
    data.throttlePos2 = analogRead(THROTTLE_POS2_PIN) / 1023.0f * 5.0f;
    data.manifoldPressure = analogRead(MAP_PIN) / 1023.0f * 5.0f;
    data.intakeTemperature = analogRead(INT_TEMP_PIN) / 1023.0f * 5.0f;
    return data;
}
