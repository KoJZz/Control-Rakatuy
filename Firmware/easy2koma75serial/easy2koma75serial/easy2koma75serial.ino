// kode easy2koma75 - Serial Input Version
#include <Arduino.h>

// Library C-SDK untuk interrupt dan PWM
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/sync.h"

// Pin definitions (Tetap sama)
#define HALL_1_PIN 13
#define HALL_2_PIN 14
#define HALL_3_PIN 15

#define AH_PIN 16
#define AL_PIN 17
#define BH_PIN 18
#define BL_PIN 19
#define CH_PIN 20
#define CL_PIN 21

const uint A_PWM_SLICE = 0; 
const uint B_PWM_SLICE = 1; 
const uint C_PWM_SLICE = 2; 

#define LED_PIN 25
#define DEADMAN_PIN 12
#define HALL_OVERSAMPLE 6
#define F_PWM 24000 

uint8_t hallToMotor[8] = {255, 1, 3, 2, 5, 0, 4, 255}; 

// Variabel global
volatile uint hall = 0;
volatile uint motorState = 0;
volatile uint8_t throttle_value = 0; // Sekarang diupdate via Serial
volatile bool deadman_active = false;

// Forward declarations
uint get_halls();
void writePWM(uint motorState, uint duty);
void writePhases(uint ah, uint bh, uint ch, uint al, uint bl, uint cl);

// ============ INTERRUPT HANDLER - PWM WRAP ============
// Sekarang semua logika komutasi dilakukan di sini setiap siklus PWM
void on_pwm_wrap() {
  pwm_clear_irq(A_PWM_SLICE); 

  // 1. Baca hall sensors
  hall = get_halls();
  motorState = hallToMotor[hall];
  
  // 2. Cek safety switch
  deadman_active = (gpio_get(DEADMAN_PIN) == LOW);
  
  // 3. Update PWM Output
  if(deadman_active) {
    writePWM(motorState, throttle_value);
  } else {
    writePWM(255, 0); // Matikan jika deadman tidak aktif
  }
}

void setup() {
  Serial.begin(115200);
  
  // Setup pins
  pinMode(HALL_1_PIN, INPUT);
  pinMode(HALL_2_PIN, INPUT);
  pinMode(HALL_3_PIN, INPUT);
  pinMode(DEADMAN_PIN, INPUT_PULLUP); // Pastikan pullup jika tidak pakai resistor eksternal
  pinMode(LED_PIN, OUTPUT);
  
  // Catatan: ADC dihapus karena input pindah ke Serial

  // ========== SETUP PWM ==========
  gpio_set_function(AH_PIN, GPIO_FUNC_PWM);
  gpio_set_function(AL_PIN, GPIO_FUNC_PWM);
  gpio_set_function(BH_PIN, GPIO_FUNC_PWM);
  gpio_set_function(BL_PIN, GPIO_FUNC_PWM);
  gpio_set_function(CH_PIN, GPIO_FUNC_PWM);
  gpio_set_function(CL_PIN, GPIO_FUNC_PWM);
  
  float pwm_divider = (float)(clock_get_hz(clk_sys)) / (F_PWM * 255 * 2);
  
  pwm_config config = pwm_get_default_config();
  pwm_config_set_clkdiv(&config, pwm_divider);
  pwm_config_set_wrap(&config, 255 - 1);
  pwm_config_set_phase_correct(&config, true);
  pwm_config_set_output_polarity(&config, false, true); 
  
  pwm_init(A_PWM_SLICE, &config, false);
  pwm_init(B_PWM_SLICE, &config, false);
  pwm_init(C_PWM_SLICE, &config, false);
  
  writePhases(0, 0, 0, 0, 0, 0);
  
  // Setup PWM interrupt
  pwm_clear_irq(A_PWM_SLICE);
  irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
  irq_set_priority(PWM_IRQ_WRAP, 0); 
  irq_set_enabled(PWM_IRQ_WRAP, true);
  
  delay(1000);
  
  pwm_set_mask_enabled(0x07); 
  pwm_set_irq_enabled(A_PWM_SLICE, true); 

  Serial.println("BLDC Controller Ready!");
  Serial.println("Kirim angka 0-255 untuk mengatur Throttle.");
}

void loop() { 
  // Membaca input dari Serial
  if (Serial.available() > 0) {
    int input = Serial.parseInt(); // Membaca angka dari Serial Monitor
    
    // Validasi input
    if (input >= 0 && input <= 255) {
      throttle_value = (uint8_t)input;
      
      Serial.print("Throttle Set ke: ");
      Serial.println(throttle_value);
    }
    
    // Bersihkan buffer sisa (seperti karakter newline)
    while(Serial.available() > 0) Serial.read();
  }

  // Indikator visual
  static unsigned long lastFlash = 0;
  if(millis() - lastFlash > 500) {
    lastFlash = millis();
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}

// --- Fungsi Pendukung (Tetap Sama) ---

void writePWM(uint motorState, uint dutyCycle) {
  uint8_t duty = (uint8_t)dutyCycle;
  uint8_t complement = 0;
  
  if (duty == 0) {
    motorState = 255;
  } else if (duty < 31) {
    complement = 0;
  } else if (duty > 250) {
    duty = 255;
    complement = 0;
  } else {
    complement = max(0, 248 - (int)duty);
  }

  if(motorState == 0)      writePhases(0, duty, 0, 255, complement, 0);
  else if(motorState == 1) writePhases(0, 0, duty, 255, 0, complement);
  else if(motorState == 2) writePhases(0, 0, duty, 0, 255, complement);
  else if(motorState == 3) writePhases(duty, 0, 0, complement, 255, 0);
  else if(motorState == 4) writePhases(duty, 0, 0, complement, 0, 255);
  else if(motorState == 5) writePhases(0, duty, 0, 0, complement, 255);
  else                     writePhases(0, 0, 0, 0, 0, 0);
}

void writePhases(uint ah, uint bh, uint ch, uint al, uint bl, uint cl) {
  pwm_set_both_levels(A_PWM_SLICE, ah, 255 - al);
  pwm_set_both_levels(B_PWM_SLICE, bh, 255 - bl);
  pwm_set_both_levels(C_PWM_SLICE, ch, 255 - cl);
}

uint get_halls() {
  uint hall_result = 0;
  // Baca langsung (oversampling disederhanakan untuk efisiensi di interrupt)
  if (gpio_get(HALL_1_PIN)) hall_result |= (1<<0);
  if (gpio_get(HALL_2_PIN)) hall_result |= (1<<1);
  if (gpio_get(HALL_3_PIN)) hall_result |= (1<<2);
  return hall_result & 0x7;
}