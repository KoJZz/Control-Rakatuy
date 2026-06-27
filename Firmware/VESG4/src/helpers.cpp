#include "helpers.h"

// Read physical status of mechanical deadman switch
bool deadmanPressed() {
    return digitalRead(PIN_DEADMAN) == LOW; 
}

// Read and compute scaled high voltage battery rail value
float readBatteryVoltage() {
    int raw_adc = analogRead(PIN_VIN_SENSE);
    float pin_voltage = (raw_adc / 4095.0f) * V_REG;
    float v_batt = pin_voltage * ((VIN_R1 + VIN_R2) / VIN_R2);
    return v_batt;
}

// Parse serial incoming numerical entries safely bounding current limits
void readSerialTarget() {
    if (Serial.available() > 0) {
        float val = Serial.parseFloat();
        while (Serial.available() > 0) Serial.read(); // Empty processing buffers

        if (val >= -MOTOR_CURRENT_LIMIT && val <= MOTOR_CURRENT_LIMIT) {
            target_current = val;
            Serial.print("Target profile updated to: ");
            Serial.println(target_current);
        } else {
            Serial.println("Entry outside parameter threshold constraints. Ignored.");
        }
    }
}