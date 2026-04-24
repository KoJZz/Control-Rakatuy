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
#define ADC_OFFSET 1971          // Nilai ADC saat arus 0A (sesuai info Anda)
#define ADC_PER_AMP 62.04       // Hasil hitung: (1mOhm * 50 gain * 4095) / 3.3V
#define HALL_OVERSAMPLE 4 
#define MAX_PWM 255
// #define Kp 0.5                  // Gain Proportional (sesuaikan jika respon terlalu kasar/lambat)

// Variabel Global
float targetCurrent = 0.0;      // Target arus dalam Ampere (dari Serial)
float currentThrottle = 0;      // Output PWM (0-255)
uint8_t hallToMotor[8] = {255, 1, 3, 2, 5, 0, 4, 255}; 
bool isRecording = false;
unsigned long startRecordTime = 0;
const unsigned long recordWindow = 1500; // Rekam selama 1.5 detik (biasanya cukup untuk respon arus)
float Kp = 0.8;    // Sesuaikan hasil analisis transfer function
float Ki = 0.15;   // Menghilangkan offset
float Kd = 0.05;   // Meredam osilasi
float integral = 0;
float lastError = 0;
unsigned long lastTime = 0;

void setup() { 
  Serial.begin(115200);
  analogReadResolution(12);     // Pastikan 12-bit untuk RP2040

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  pinMode(AH_PIN, OUTPUT); pinMode(AL_PIN, OUTPUT);
  pinMode(BH_PIN, OUTPUT); pinMode(BL_PIN, OUTPUT);
  pinMode(CH_PIN, OUTPUT); pinMode(CL_PIN, OUTPUT);
  analogWriteFreq(24000); 

  pinMode(HALL_1_PIN, INPUT); pinMode(HALL_2_PIN, INPUT); pinMode(HALL_3_PIN, INPUT);
  pinMode(DEADMAN_PIN, INPUT); 
  
  delay(2000);
  Serial.println("Current Control Ready. Masukkan target arus (Ampere), contoh: 2.5");
}

float readCurrentAmps() {
  int raw = analogRead(CURRENT_ADC_PIN);
  // Arus = (Nilai_Sekarang - Offset) / (ADC_per_Ampere)
  return (float)(raw - ADC_OFFSET) / ADC_PER_AMP;
  //return (float)(raw);
}

void loop() { 
  // 1. Baca input Serial (Trigger)
  if (Serial.available() > 0) {
    float inputVal = Serial.parseFloat();
    if (inputVal >= 0) {
      targetCurrent = inputVal;
      isRecording = true;
      startRecordTime = millis();
      Serial.println("TIME,TARGET,ACTUAL");
    }
    while(Serial.available() > 0) Serial.read();
  }

  // 2. Hitung Waktu dan Error
  unsigned long currentTime = micros(); // Gunakan micros untuk presisi
  float dt = (float)(currentTime - lastTime) / 1000000.0;
  if (dt <= 0) dt = 0.0001; 

  float actualCurrent = readCurrentAmps();
  float error = targetCurrent - actualCurrent;

  // 3. Algoritma PID
  float P_out = Kp * error;
  
  // Anti-Windup: Hanya tambah integral jika output belum mentok
  if (currentThrottle > 0 && currentThrottle < MAX_PWM) {
    integral += error * dt;
  }
  float I_out = Ki * integral;

  float D_out = Kd * (error - lastError) / dt;
  float totalOutput = P_out + I_out + D_out;

  // 4. Update Variabel State
  currentThrottle = totalOutput;
  lastError = error;
  lastTime = currentTime;

  // 5. Batasi PWM dan Safety 
  if (currentThrottle > MAX_PWM) currentThrottle = MAX_PWM; 
  if (currentThrottle < 0) currentThrottle = 0; 
  
  if (targetCurrent == 0) {
    currentThrottle = 0;
    integral = 0; 
  }

  // 6. Komutasi Motor (PENTING: Harus ada agar motor berputar)
  uint8_t hall = getHalls();
  uint8_t motorState = hallToMotor[hall];
  writePWM(motorState, (uint8_t)currentThrottle);

  // 7. Logging CSV
  if (isRecording) {
    unsigned long elapsed = millis() - startRecordTime;
    if (elapsed < recordWindow) {
      Serial.print(elapsed);
      Serial.print(",");
      Serial.print(targetCurrent);
      Serial.print(",");
      Serial.println(actualCurrent);
    } else {
      isRecording = false;
      Serial.println("---END---");
    }
  }
}

// --- FUNGSI LOW LEVEL (TETAP SAMA) ---

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