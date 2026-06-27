#pragma once
#include <Arduino.h>
#include "board_config.h"
#include "motor_config.h"

// Share target current across source scopes
extern float target_current;
extern bool deadman_bypass;

// Helper utilities function declarations
bool deadmanPressed();
float readBatteryVoltage();
void readSerialTarget();