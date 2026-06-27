#pragma once
#include <Arduino.h>

class DRV8302 {
public:
    DRV8302(int enGatePin, int dcCalPin, int faultPin);
    void init();
    void enableGate();
    void disableGate();
    bool isFaulted();  // true if fault line is active (active-low signal)

private:
    int _enGatePin;
    int _dcCalPin;
    int _faultPin;
};