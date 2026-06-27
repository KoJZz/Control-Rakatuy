#include "drv8302.h"

DRV8302::DRV8302(int enGatePin, int dcCalPin, int faultPin)
    : _enGatePin(enGatePin), _dcCalPin(dcCalPin), _faultPin(faultPin) {}

void DRV8302::init() {
    pinMode(_enGatePin, OUTPUT);
    digitalWrite(_enGatePin, LOW);  // start disabled, mirrors VESC's DISABLE_GATE()

    pinMode(_dcCalPin, OUTPUT);
    digitalWrite(_dcCalPin, LOW);

    pinMode(_faultPin, INPUT_PULLUP);
}

void DRV8302::enableGate() {
    digitalWrite(_enGatePin, HIGH);
}

void DRV8302::disableGate() {
    digitalWrite(_enGatePin, LOW);
}

bool DRV8302::isFaulted() {
    // Active-low fault signal, matches IS_DRV_FAULT() macro from hw_410.h
    return digitalRead(_faultPin) == LOW;
}