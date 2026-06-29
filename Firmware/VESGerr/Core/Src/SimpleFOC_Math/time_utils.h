#pragma once
#include "stm32f4xx_hal.h"

// Satisfy pid.cpp's internal call to _micros() using the native STM32 HAL 1ms tick
static inline unsigned long _micros() {
    return HAL_GetTick() * 1000;
}
