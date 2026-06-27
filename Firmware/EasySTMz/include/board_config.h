#pragma once

// ===== PWM (Gate driver) pins - TIM1 channels =====
#define PWM_AH PA10
#define PWM_AL PB15

#define PWM_BH PA9
#define PWM_BL PB14

#define PWM_CH PA8
#define PWM_CL PB13

// ===== DRV8302 control pins =====
#define PIN_EN_GATE   PC10
#define PIN_DCCAL     PB12
#define PIN_DRV_FAULT PC12

// ===== Hall sensor pins =====
#define PIN_HALL_A PB6
#define PIN_HALL_B PB7
#define PIN_HALL_C PC11

// ===== Current sense pins =====
#define PIN_CURR_A PB1   // SH1 -> SN1/SP1 -> BR_SO1 -> CURR1 -> Phase A (PHASE_1, Q1/Q2)
#define PIN_CURR_C PB0   // SH2 -> SN2/SP2 -> BR_SO2 -> CURR2 -> Phase C (PHASE_3, Q5/Q6)

// ===== Voltage sense =====
#define PIN_VIN_SENSE PC2

// ===== LEDs =====
#define PIN_LED_GREEN PC4
#define PIN_LED_RED   PC5

// ===== Deadman switch (active-LOW, external pull-up on board) =====
#define PIN_DEADMAN PA6

// ===== Throttle (analog, Vcc-referenced) =====
#define PIN_THROTTLE PA5

// ===== Board electrical constants (from hw_410.h) =====
#define CURRENT_SHUNT_RES   0.001f   // 1 mOhm
#define CURRENT_AMP_GAIN    10.0f
#define VIN_R1              39000.0f
#define VIN_R2              2200.0f
#define V_REG               3.3f
#define PWM_FREQ            20000