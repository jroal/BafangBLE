#pragma once

#include <cstdint>

// Plain data snapshot shared between the hardware firmware and the PC simulator.
struct MotorMetrics {
    float speedKmh = 0.0f;
    uint16_t cadenceRpm = 0;
    uint16_t motorPowerWatts = 0;
    uint8_t assistLevel = 0;
    int8_t motorTempC = 0;
    float batteryVoltage = 0.0f;
    float batteryCurrent = 0.0f;
    uint8_t batterySocPercent = 0;
    uint8_t errorCode = 0;
    bool canActive = false;
};
