#include <digitalWriteFast.h>
#include "hardware/gpio.h"
#include "hardware/timer.h" // Library hardware timer RP2040

// --- KONFIGURASI PIN ---
#define HALL_1_PIN 13
#define HALL_2_PIN 14
#define HALL_3_PIN 15
#define AH_PIN 16 
#define AL_PIN 17
#define BH_PIN 18
#define BL_PIN 19
#define CH_PIN 20
#define CL_PIN 21
#define LED_PIN 25 
#define DEADMAN_PIN 12
#define CURRENT_ADC_PIN 26

// --- PARAMETER SENSOR & CONTROL ---
#define ADC_OFFSET 1971          
#define ADC_PER_AMP 62.04       
#define HALL_OVERSAMPLE 4 
#define MAX_PWM 255

// --- KONSTANTA PID ---
float Kp = 0.07;           // Mulai dari nilai kecil
float Ki = 0.5;           // Menghilangkan steady-state error
float integralError = 0;  // Akumulator error
const float dt = 0.001;   // dt tetap 1ms (0.001 detik)

// --- VARIABEL GLOBAL ---
volatile float targetCurrent = 0.0;
volatile float actualCurrent_global = 0.0;
volatile float currentThrottle = 0;
uint8_t hallToMotor[8] = {255, 1, 3, 2, 5, 0, 4, 255}; 

// --- VARIABEL LOGGING ---
volatile bool isRecording = false;
unsigned long startRecordTime = 0;
const unsigned long recordWindow = 1500; 

struct repeating_timer timer; // Struct untuk handle timer hardware

// ================================================================
// FUNCTION: Hardware Timer Callback (Berjalan otomatis setiap 1ms)
// ================================================================
bool controlLoopCallback(struct repeating_timer *t) {
    // 1. Baca Arus Aktual
    int raw = analogRead(CURRENT_ADC_PIN);
    actualCurrent_global = (float)(raw - ADC_OFFSET) / ADC_PER_AMP;

    // 2. Hitung Error
    float error = targetCurrent - actualCurrent_global;

    // 3. Algoritma PI Control
    float P_out = Kp * error;

    // Anti-Windup: Integral hanya bertambah jika PWM tidak saturasi
    if (currentThrottle < MAX_PWM && currentThrottle > 0) {
        integralError += error * dt;
    }
    float I_out = Ki * integralError;

    // 4. Update Output
    float output = P_out + I_out;

    // Limitasi & Safety
    if (output > MAX_PWM) output = MAX_PWM;
    if (output < 0) output = 0;
    
    // Safety: Jika target nol atau tombol deadman dilepas (HIGH)
    if (targetCurrent <= 0.01 || gpio_get(DEADMAN_PIN)) {
        output = 0;
        integralError = 0; // Reset integral saat mati
    }

    currentThrottle = output;

    // 5. Eksekusi Komutasi (Langsung di dalam interrupt untuk respon tercepat)
    uint8_t hall = 0;
    // Baca Hall Sensor
    if (gpio_get(HALL_1_PIN)) hall |= (1<<0);
    if (gpio_get(HALL_2_PIN)) hall |= (1<<1);
    if (gpio_get(HALL_3_PIN)) hall |= (1<<2);
    
    uint8_t motorState = hallToMotor[hall & 0x7];
    writePWM(motorState, (uint8_t)currentThrottle);

    return true; // Lanjutkan timer
}

void setup() { 
    Serial.begin(115200);
    analogReadResolution(12);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    pinMode(AH_PIN, OUTPUT); pinMode(AL_PIN, OUTPUT);
    pinMode(BH_PIN, OUTPUT); pinMode(BL_PIN, OUTPUT);
    pinMode(CH_PIN, OUTPUT); pinMode(CL_PIN, OUTPUT);
    analogWriteFreq(24000); 

    pinMode(HALL_1_PIN, INPUT); pinMode(HALL_2_PIN, INPUT); pinMode(HALL_3_PIN, INPUT);
    pinMode(DEADMAN_PIN, INPUT_PULLUP); 
    
    delay(2000);

    // INISIALISASI TIMER: Jalankan controlLoopCallback setiap 1ms (-1000us)
    add_repeating_timer_us(-1000, controlLoopCallback, NULL, &timer);
    
    Serial.println("Hardware Timer PI Ready. Input target (A):");
}

void loop() { 
    // Loop utama HANYA menangani Komunikasi dan Logging
    if (Serial.available() > 0) {
        float inputVal = Serial.parseFloat(); 
        if (inputVal >= 0) {
            targetCurrent = inputVal;
            isRecording = true;
            startRecordTime = millis();
            Serial.println("TIME(ms),TARGET(A),ACTUAL(A),PWM");
        }
        while(Serial.available() > 0) Serial.read();
    }

    if (isRecording) {
        unsigned long elapsed = millis() - startRecordTime;
        if (elapsed < recordWindow) {
            Serial.print(elapsed); Serial.print(",");
            Serial.print(targetCurrent); Serial.print(",");
            Serial.print(actualCurrent_global); Serial.print(",");
            Serial.println((uint8_t)currentThrottle);
        } else {
            isRecording = false;
            Serial.println("---END_OF_DATA---");
        }
    }
}

// --- FUNGSI LOW LEVEL ---
void writePWM(uint8_t motorState, uint8_t dutyCycle) {
    if(dutyCycle == 0 || motorState == 255) {
        writePhases(0, 0, 0, 0, 0, 0);
        return;
    }
    if(motorState == 0)      writePhases(0, dutyCycle, 0, 1, 0, 0);
    else if(motorState == 1) writePhases(0, 0, dutyCycle, 1, 0, 0);
    else if(motorState == 2) writePhases(0, 0, dutyCycle, 0, 1, 0);
    else if(motorState == 3) writePhases(dutyCycle, 0, 0, 0, 1, 0);
    else if(motorState == 4) writePhases(dutyCycle, 0, 0, 0, 0, 1);
    else if(motorState == 5) writePhases(0, dutyCycle, 0, 0, 0, 1);
}

void writePhases(uint8_t ah, uint8_t bh, uint8_t ch, uint8_t al, uint8_t bl, uint8_t cl) {
    analogWrite(AH_PIN, ah);
    analogWrite(BH_PIN, bh);
    analogWrite(CH_PIN, ch);
    digitalWriteFast(AL_PIN, al);
    digitalWriteFast(BL_PIN, bl);
    digitalWriteFast(CL_PIN, cl);
}