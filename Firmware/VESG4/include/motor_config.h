#pragma once

// ===== Motor-specific parameters =====
#define MOTOR_POLE_PAIRS     8     

// ===== OPERATION MODE =====
// Uncomment for safe bench testing. Comment out for actual driving.
#define BENCH_TEST_MODE 

// ===== SAFETY LIMITS =====
#ifdef BENCH_TEST_MODE
    // Safe ceilings for the bench supply setup
    #define DRIVER_VOLTAGE_LIMIT 6.0f
    #define MOTOR_CURRENT_LIMIT  0.5f
#else
    // Full power battery limitations
    #define DRIVER_VOLTAGE_LIMIT 54.6f   
    #define MOTOR_CURRENT_LIMIT  20.0f   
#endif

#define FOC_PWM_FREQ 20000