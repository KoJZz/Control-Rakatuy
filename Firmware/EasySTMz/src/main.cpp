#include <Arduino.h>
#include "board_config.h"
#include "drv8302.h"

// ===== Throttle scaling (from original Pico sketch, unused unless you wire analog throttle later) =====
#define THROTTLE_LOW  4
#define THROTTLE_HIGH 741

#define HALL_OVERSAMPLE 4

// Sequence Mitsuba Clockwise
uint8_t hallToMotor[8] = {255, 1, 3, 2, 5, 0, 4, 255};

DRV8302 drv(PIN_EN_GATE, PIN_DCCAL, PIN_DRV_FAULT);

// Timer for non-blocking periodic debug prints
uint32_t lastDebugPrint = 0;
uint32_t faultResetCount = 0; // Tracks historical frequency of hard resets

// ===== Forward declarations =====
void writePhases(uint8_t ah, uint8_t bh, uint8_t ch, uint8_t al, uint8_t bl, uint8_t cl);
void writePWM(uint8_t motorState, uint8_t dutyCycle);
uint8_t getHalls();
uint8_t readThrottle();
float readBatteryVoltage();
bool deadmanPressed();

void setup() {
    Serial.begin(115200);

    pinMode(PIN_LED_GREEN, OUTPUT);
    pinMode(PIN_LED_RED, OUTPUT);
    digitalWrite(PIN_LED_RED, HIGH); // red while booting

    uint32_t timeout = millis();
    while (!Serial && (millis() - timeout < 3000)) { delay(10); }
    Serial.println("--- BLDC SIX-STEP DEBUG FIRMWARE ---");
    Serial.println("USB Serial connected. Initializing hardware...");

    analogReadResolution(12);

    // ===== Wait for real PSU voltage before doing anything with the DRV =====
    Serial.println("Waiting for High Voltage power supply...");
    float v_bat = readBatteryVoltage();
    while (v_bat < 8.0f) {
        Serial.print("Low Voltage Halt! Detected: ");
        Serial.print(v_bat);
        Serial.println("V (Target: >8.0V)");
        
        digitalWrite(PIN_LED_RED, !digitalRead(PIN_LED_RED));
        delay(500);
        v_bat = readBatteryVoltage();
    }
    Serial.print("SUCCESS: High Voltage detected at ");
    Serial.print(v_bat);
    Serial.println("V");
    digitalWrite(PIN_LED_RED, LOW);

    // ===== Bring up DRV8302 deliberately, same pattern as the FOC sketch =====
    drv.init();
    drv.disableGate(); // clear any transient wake-up faults
    delay(50);

    pinMode(PWM_AH, OUTPUT);
    pinMode(PWM_AL, OUTPUT);
    pinMode(PWM_BH, OUTPUT);
    pinMode(PWM_BL, OUTPUT);
    pinMode(PWM_CH, OUTPUT);
    pinMode(PWM_CL, OUTPUT);

    // All outputs explicitly OFF before enabling the gate driver
    writePhases(0, 0, 0, 0, 0, 0);

    analogWriteFrequency(PWM_FREQ);

    pinMode(PIN_HALL_A, INPUT);
    pinMode(PIN_HALL_B, INPUT);
    pinMode(PIN_HALL_C, INPUT);
    pinMode(PIN_DEADMAN, INPUT); // physical pull-up on board, but bypassed in software now

    drv.enableGate();
    delay(100);

    // ===== FAULT BYPASS IN SETUP =====
    if (drv.isFaulted()) {
        faultResetCount++;
        Serial.println("WARNING: DRV is throwing a hard fault at setup, but bypassing as requested.");
        Serial.println(faultResetCount);
        digitalWrite(PIN_LED_RED, HIGH);
    }

    Serial.println("Setup complete. Ready for serial throttle inputs (0-255)...");
}

void loop() {
    // ===== Fault check - Non-blocking Auto-Reset =====
    if (drv.isFaulted()) {
        faultResetCount++;
        digitalWrite(PIN_LED_RED, HIGH);
        Serial.println("!!! DRV FAULT PIN ACTIVE !!! Attempting hard reset on EN_GATE...");
        
        // Pulse EN_GATE to try and clear latched hardware conditions
        drv.disableGate();
        delayMicroseconds(20); 
        drv.enableGate();
    } else {
        digitalWrite(PIN_LED_RED, LOW);
    }

    // ===== Deadman check (Bypassed) =====
    if (!deadmanPressed()) {
        writePWM(255, 0); // force all phases off
        digitalWrite(PIN_LED_GREEN, LOW);
        return;
    }

    uint8_t throttle = readThrottle();

    // High speed commutation sub-loop
    uint32_t currentAccumulatorPB0 = 0;
    uint32_t currentAccumulatorPB1 = 0;

    for (uint8_t i = 0; i < 200; i++) {
        uint8_t hall = getHalls();
        uint8_t motorState = hallToMotor[hall];
        writePWM(motorState, throttle);

        // Keep accumulating raw values at raw hardware speeds
        currentAccumulatorPB0 += analogRead(PIN_CURR_C);
        currentAccumulatorPB1 += analogRead(PIN_CURR_A);
    }

    // Capture the averaged result of those 200 high-speed iterations
    int activeCurrentPB0 = currentAccumulatorPB0 / 200;
    int activeCurrentPB1 = currentAccumulatorPB1 / 200;

    digitalWrite(PIN_LED_GREEN, throttle > 0 ? HIGH : LOW);
    // ===== High-Density Telemetry Logging (Every 500ms) =====
    if (millis() - lastDebugPrint >= 500) {
        lastDebugPrint = millis();
        
        uint8_t currentHall = getHalls();

        Serial.print("[DATA] Throttle: ");
        Serial.print(throttle);
        Serial.print(" | PB0_Raw: ");
        Serial.print(activeCurrentPB0);
        Serial.print(" | PB1_Raw: ");
        Serial.print(activeCurrentPB1);
        Serial.print(" | Bus: ");
        Serial.print(readBatteryVoltage(), 2);
        Serial.print("V | Resets: ");
        Serial.println(faultResetCount);
    }
}

void writePWM(uint8_t motorState, uint8_t dutyCycle) {
    if (dutyCycle == 0)
        motorState = 255;

    if (motorState == 0)
        writePhases(0, dutyCycle, 0, 1, 0, 0);
    else if (motorState == 1)
        writePhases(0, 0, dutyCycle, 1, 0, 0);
    else if (motorState == 2)
        writePhases(0, 0, dutyCycle, 0, 1, 0);
    else if (motorState == 3)
        writePhases(dutyCycle, 0, 0, 0, 1, 0);
    else if (motorState == 4)
        writePhases(dutyCycle, 0, 0, 0, 0, 1);
    else if (motorState == 5)
        writePhases(0, dutyCycle, 0, 0, 0, 1);
    else
        writePhases(0, 0, 0, 0, 0, 0);
}

void writePhases(uint8_t ah, uint8_t bh, uint8_t ch, uint8_t al, uint8_t bl, uint8_t cl) {
    analogWrite(PWM_AH, ah);
    analogWrite(PWM_BH, bh);
    analogWrite(PWM_CH, ch);
    // Low sides run as completely independent GPIO switches to prevent timer register clashes
    digitalWrite(PWM_AL, al);
    digitalWrite(PWM_BL, bl);
    digitalWrite(PWM_CL, cl);
}


uint8_t getHalls() {
    uint8_t hallCounts[] = {0, 0, 0};
    for (uint8_t i = 0; i < HALL_OVERSAMPLE; i++) {
        hallCounts[0] += digitalRead(PIN_HALL_A);
        hallCounts[1] += digitalRead(PIN_HALL_B);
        hallCounts[2] += digitalRead(PIN_HALL_C);
    }

    uint8_t hall = 0;
    if (hallCounts[0] >= HALL_OVERSAMPLE / 2) hall |= (1 << 0);
    if (hallCounts[1] >= HALL_OVERSAMPLE / 2) hall |= (1 << 1);
    if (hallCounts[2] >= HALL_OVERSAMPLE / 2) hall |= (1 << 2);

    return hall & 0x7;
}

uint8_t readThrottle() {
    static uint8_t currentThrottle = 0;

    if (Serial.available() > 0) {
        int serialInput = Serial.parseInt();
        while (Serial.available() > 0) Serial.read();

        if (serialInput > 255) currentThrottle = 255;
        else if (serialInput < 0) currentThrottle = 0;
        else currentThrottle = serialInput;
        
        Serial.print("-> New Throttle Target Set: ");
        Serial.println(currentThrottle);
    }

    return currentThrottle;
}

bool deadmanPressed() {
    // ===== DEADMAN BYPASS ACTIVE =====
    // Force true so you do not need physical hardware pulled low to test
    return true; 
    
    // Original Code: return digitalRead(PIN_DEADMAN) == LOW; 
}

float readBatteryVoltage() {
    int raw = analogRead(PIN_VIN_SENSE);
    float adc_voltage = (raw / 4095.0f) * V_REG;
    return adc_voltage * ((VIN_R1 + VIN_R2) / VIN_R2);
}