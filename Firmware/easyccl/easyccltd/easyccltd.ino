#include <digitalWriteFast.h>
#include "hardware/gpio.h"

// --- KONFIGURASI PIN ---
#define THROTTLE_PIN 28

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
#define Kp 0.07                  

#define MAX_TARGET_CURRENT 5.0   // Tentukan batas maksimal potensio (Ampere)

// VARIABEL GLOBAL 
float targetCurrent = 0.0;
float currentThrottle = 0;      
uint8_t hallToMotor[8] = {255, 1, 3, 2, 5, 0, 4, 255};

// VARIABEL LOGGING 
/*
bool isRecording = false;
unsigned long startRecordTime = 0;
const unsigned long recordWindow = 3000;
*/

void setup() { 
  // Inisialisasi Serial tetap menyala jika Anda ingin menghidupkan telemetri
  Serial.begin(115200); 
  analogReadResolution(12);     

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  pinMode(AH_PIN, OUTPUT); pinMode(AL_PIN, OUTPUT);
  pinMode(BH_PIN, OUTPUT); pinMode(BL_PIN, OUTPUT);
  pinMode(CH_PIN, OUTPUT); pinMode(CL_PIN, OUTPUT);
  analogWriteFreq(24000); 

  pinMode(HALL_1_PIN, INPUT); pinMode(HALL_2_PIN, INPUT); pinMode(HALL_3_PIN, INPUT);
  
  pinMode(THROTTLE_PIN, INPUT);
  pinMode(DEADMAN_PIN, INPUT_PULLUP); 
  
  delay(2000);
  // Serial.println("System Ready. Mode Potensio Aktif.");
}

float readCurrentAmps() {
  int raw = analogRead(CURRENT_ADC_PIN);
  return (float)(raw - ADC_OFFSET) / ADC_PER_AMP;
}

void loop() { 
  // --- 1A. LOGIKA INPUT SERIAL (DINONAKTIFKAN) ---
  /*
  if (Serial.available() > 0) {
    float inputVal = Serial.parseFloat();
    if (inputVal >= 0) {
      targetCurrent = inputVal;
      isRecording = true;
      startRecordTime = millis();
      Serial.println("TIME(ms),TARGET(A),ACTUAL(A),PWM,HALL");
    }
    while(Serial.available() > 0) Serial.read();
  }
  */

  // BACA POTENSIO (AKTIF)
  int potValue = analogRead(THROTTLE_PIN);
  targetCurrent = ((float)potValue / 4095.0) * MAX_TARGET_CURRENT;

  // DEADMAN SWITCH
  if (digitalRead(DEADMAN_PIN) == HIGH) {
    targetCurrent = 0.0; 
  }

  // BACA ARUS AKTUAL
  float actualCurrent = readCurrentAmps();

  // KONTROL ARUS (P-Control)
  float error = targetCurrent - actualCurrent;
  currentThrottle += (error * Kp);

  // BATASI PWM & SAFETY
  if (currentThrottle > MAX_PWM) currentThrottle = MAX_PWM;
  if (currentThrottle < 0) currentThrottle = 0;
  
  if (targetCurrent <= 0.01) {
    currentThrottle = 0;
  }

  // KOMUTASI MOTOR 
  uint8_t hall = getHalls();
  uint8_t motorState = hallToMotor[hall]; 
  writePWM(motorState, (uint8_t)currentThrottle);         

  // HIGH-SPEED LOGGING CSV (DINONAKTIFKAN)
  /*
  if (isRecording) {
    unsigned long elapsed = millis() - startRecordTime;
    if (elapsed < recordWindow) {
      Serial.print(elapsed); Serial.print(",");
      Serial.print(targetCurrent); Serial.print(",");
      Serial.print(actualCurrent); Serial.print(",");
      Serial.print((uint8_t)currentThrottle); Serial.print(",");
      Serial.println(hall);
    } else {
      isRecording = false;
      Serial.println("---END_OF_DATA---");
    }
  }
  */

  // TELEMETRI RINGAN 200ms
  // Jika ingin melihat nilai potensio/arus berjalan tanpa nge-log CSV
  /*
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    Serial.print("Target(A): "); Serial.print(targetCurrent);
    Serial.print(" | Aktual(A): "); Serial.print(actualCurrent);
    Serial.print(" | PWM: "); Serial.println((uint8_t)currentThrottle);
    lastPrint = millis();
  }
  */
}

// --- FUNGSI LOW LEVEL ---

void writePWM(uint8_t motorState, uint8_t dutyCycle) {
  if(dutyCycle == 0) motorState = 255;
  if(motorState == 0)      writePhases(0, dutyCycle, 0, 1, 0, 0);
  else if(motorState == 1) writePhases(0, 0, dutyCycle, 1, 0, 0);
  else if(motorState == 2) writePhases(0, 0, dutyCycle, 0, 1, 0);
  else if(motorState == 3) writePhases(dutyCycle, 0, 0, 0, 1, 0);
  else if(motorState == 4) writePhases(dutyCycle, 0, 0, 0, 0, 1);
  else if(motorState == 5) writePhases(0, dutyCycle, 0, 0, 0, 1);
  else                     writePhases(0, 0, 0, 0, 0, 0);
}

void writePhases(uint8_t ah, uint8_t bh, uint8_t ch, uint8_t al, uint8_t bl, uint8_t cl) {
  analogWrite(AH_PIN, ah);
  analogWrite(BH_PIN, bh);
  analogWrite(CH_PIN, ch);
  digitalWriteFast(AL_PIN, al);
  digitalWriteFast(BL_PIN, bl);
  digitalWriteFast(CL_PIN, cl);
}

uint8_t getHalls() {
  uint8_t hallCounts[] = {0, 0, 0};
  for(uint8_t i = 0; i < HALL_OVERSAMPLE; i++) {
    hallCounts[0] += gpio_get(HALL_1_PIN);
    hallCounts[1] += gpio_get(HALL_2_PIN);
    hallCounts[2] += gpio_get(HALL_3_PIN);
  }
  uint8_t hall = 0;
  if (hallCounts[0] >= (HALL_OVERSAMPLE / 2)) hall |= (1<<0);
  if (hallCounts[1] >= (HALL_OVERSAMPLE / 2)) hall |= (1<<1);
  if (hallCounts[2] >= (HALL_OVERSAMPLE / 2)) hall |= (1<<2);
  return hall & 0x7;
}