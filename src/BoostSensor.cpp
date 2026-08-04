#include "BoostSensor.h"

// Pin assignments – TBD, replace with actual pin numbers
const uint8_t THROTTLE_POS1_PIN = A0;
const uint8_t THROTTLE_POS2_PIN = A1;
const uint8_t MAP_PIN          = A2;
const uint8_t INT_TEMP_PIN     = A3;

void BoostSensor_init() {
    // Configure ADC pins as inputs (Arduino style)
    pinMode(THROTTLE_POS1_PIN, INPUT);
    pinMode(THROTTLE_POS2_PIN, INPUT);
    pinMode(MAP_PIN,           INPUT);
    pinMode(INT_TEMP_PIN,      INPUT);
}

BoostSensorData BoostSensor_read() {
    BoostSensorData data;
    data.throttlePos1 = analogRead(THROTTLE_POS1_PIN) / 1023.0f * 5.0f; // example scaling
    data.throttlePos2 = analogRead(THROTTLE_POS2_PIN) / 1023.0f * 5.0f;
    data.manifoldPressure = analogRead(MAP_PIN) / 1023.0f * 5.0f;
    data.intakeTemperature = analogRead(INT_TEMP_PIN) / 1023.0f * 5.0f;
    return data;
}
