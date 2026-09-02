#pragma once

// Sensor read functions for the boost controller
#include <Arduino.h>

struct BoostSensorData {
    float throttlePos1;   // Throttle position feedback 1 (e.g., potentiometer)
    float throttlePos2;   // Throttle position feedback 2
    float manifoldPressure; // MAP sensor reading
    float intakeTemperature; // Intake temperature sensor
};

void MngBoostSensor_Init();
BoostSensorData MngBoostSensor_Read();
