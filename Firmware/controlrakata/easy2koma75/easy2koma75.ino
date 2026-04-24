// kode easy2koma75

#include <Arduino.h>
#include <digitalWriteFast.h>

// Library C-SDK untuk interrupt dan PWM
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/sync.h"

#define THROTTLE_PIN 28
#define THROTTLE_LOW 16
#define THROTTLE_HIGH 2950

#define HALL_1_PIN 13
#define HALL_2_PIN 14
#define HALL_3_PIN 15

#define AH_PIN 16
#define AL_PIN 17
#define BH_PIN 18
#define BL_PIN 19
#define CH_PIN 20
#define CL_PIN 21

// Slice PWM untuk 3 pasang pin
const uint A_PWM_SLICE = 0; // Pin 16 & 17
const uint B_PWM_SLICE = 1; // Pin 18 & 19
const uint C_PWM_SLICE = 2; // Pin 20 & 21

#define LED_PIN 25
#define DEADMAN_PIN 12

#define HALL_OVERSAMPLE 6
#define F_PWM 24000 // Frekuensi 24kHz

uint8_t hallToMotor[8] = {255, 1, 3, 2, 5, 0, 4, 255}; // Sequence Mitsuba Clockwise

// Variabel global untuk interrupt
volatile int adc_throttle = 0;
volatile int fifo_level = 0;
volatile uint hall = 0;
volatile uint motorState = 0;
volatile uint8_t throttle_value = 0;
volatile bool deadman_active = false;

// Forward declarations
uint get_halls();
void writePWM(uint motorState, uint duty);
void writePhases(uint ah, uint bh, uint ch, uint al, uint bl, uint cl);
//uint8_t readThrottle();

// ============ INTERRUPT HANDLER - PWM WRAP ============
void on_pwm_wrap() {
  // Interrupt ini dipicu saat PWM slice A mencapai 0 (tengah siklus PWM)
  // Kita gunakan untuk memulai pembacaan ADC throttle
  
  pwm_clear_irq(A_PWM_SLICE); // Clear interrupt flag
  
  // Mulai konversi ADC untuk throttle
  adc_select_input(2); // Throttle di ADC2 (pin 28)
  adc_run(true);
}

// ============ INTERRUPT HANDLER - ADC FIFO ============
void on_adc_fifo() {
  // Interrupt ini dipicu saat ADC selesai konversi
  // Di sini kita baca hall sensor dan update motor state
  
  uint32_t flags = save_and_disable_interrupts(); // Disable interrupt sementara
  
  adc_run(false); // Stop ADC
  
  // Baca ADC throttle dari FIFO
  fifo_level = adc_fifo_get_level();
  if(fifo_level > 0) {
    adc_throttle = adc_fifo_get();
  }
  
  // Clear FIFO jika masih ada sisa
  while(!adc_fifo_is_empty()) {
    adc_fifo_get();
  }
  
  restore_interrupts(flags); // Re-enable interrupts
  
  // Validasi FIFO level
  if(fifo_level != 1) {
    return; // Abort jika tidak sesuai ekspektasi
  }
  
  // Baca hall sensors
  hall = get_halls();
  motorState = hallToMotor[hall];
  
  // Konversi throttle ADC ke 0-255
  int32_t throttle_calc = ((adc_throttle - THROTTLE_LOW) * 256) / (THROTTLE_HIGH - THROTTLE_LOW);
  throttle_calc = max(0, min(255, throttle_calc));
  throttle_value = (uint8_t)throttle_calc;
  
  // Update PWM berdasarkan deadman switch
  deadman_active = (digitalRead(DEADMAN_PIN) == LOW);
  
  if(deadman_active) {
    writePWM(motorState, throttle_value);
  } else {
    writePWM(255, 0); // All off
  }
}

void setup() {
  Serial.begin(115200);
  
  // Setup pins
  pinMode(HALL_1_PIN, INPUT);
  pinMode(HALL_2_PIN, INPUT);
  pinMode(HALL_3_PIN, INPUT);
  pinMode(THROTTLE_PIN, INPUT);
  pinMode(DEADMAN_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  
  // ========== SETUP ADC ==========
  adc_init();
  adc_gpio_init(THROTTLE_PIN);
  
  sleep_ms(100);
  
  // Setup ADC FIFO dan interrupt
  adc_fifo_setup(
    true,   // Write each conversion to FIFO
    false,  // Don't enable DMA
    1,      // Threshold = 1 (interrupt setelah 1 sample)
    false,  // No error bit
    false   // Don't shift samples
  );
  
  // Setup ADC interrupt
  irq_set_exclusive_handler(ADC_IRQ_FIFO, on_adc_fifo);
  irq_set_priority(ADC_IRQ_FIFO, 0); // Highest priority
  adc_irq_set_enabled(true);
  irq_set_enabled(ADC_IRQ_FIFO, true);
  
  // ========== SETUP PWM ==========
  gpio_set_function(AH_PIN, GPIO_FUNC_PWM);
  gpio_set_function(AL_PIN, GPIO_FUNC_PWM);
  gpio_set_function(BH_PIN, GPIO_FUNC_PWM);
  gpio_set_function(BL_PIN, GPIO_FUNC_PWM);
  gpio_set_function(CH_PIN, GPIO_FUNC_PWM);
  gpio_set_function(CL_PIN, GPIO_FUNC_PWM);
  
  // Hitung PWM divider untuk 24kHz
  float pwm_divider = (float)(clock_get_hz(clk_sys)) / (F_PWM * 255 * 2);
  
  // Konfigurasi PWM
  pwm_config config = pwm_get_default_config();
  pwm_config_set_clkdiv(&config, pwm_divider);
  pwm_config_set_wrap(&config, 255 - 1);
  pwm_config_set_phase_correct(&config, true);
  pwm_config_set_output_polarity(&config, false, true); // Invert low-side
  
  // Initialize semua PWM slices
  pwm_init(A_PWM_SLICE, &config, false);
  pwm_init(B_PWM_SLICE, &config, false);
  pwm_init(C_PWM_SLICE, &config, false);
  
  // Set semua PWM ke 0 dulu
  writePhases(0, 0, 0, 0, 0, 0);
  
  // Setup PWM interrupt untuk slice A
  pwm_clear_irq(A_PWM_SLICE);
  irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
  irq_set_priority(PWM_IRQ_WRAP, 0); // Highest priority
  irq_set_enabled(PWM_IRQ_WRAP, true);
  
  delay(2000);
  
  // Enable PWM slices dan interrupt
  pwm_set_mask_enabled(0x07); // Enable slice 0, 1, 2
  pwm_set_irq_enabled(A_PWM_SLICE, true); // Enable interrupt untuk slice A
  
  Serial.println("Controller initialized with interrupt system");
  Serial.println("Hall Table (hallToMotor):");
  for(uint8_t i = 0; i < 8; i++) {
    Serial.print(hallToMotor[i]);
    Serial.print(", ");
  }
  Serial.println();
}

void loop() { 
  // Loop sekarang hanya untuk monitoring dan debugging
  // Semua kontrol motor dilakukan di interrupt handler
  
  // static unsigned long lastPrint = 0;
  // if(millis() - lastPrint > 100) {
  //   lastPrint = millis();
    
  //   // Toggle LED untuk indikasi system running
  //   digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    
  //   // Print status untuk debugging
  //   Serial.print("Throttle: ");
  //   Serial.print(throttle_value);
  //   Serial.print(" | Hall: ");
  //   Serial.print(hall);
  //   Serial.print(" | State: ");
  //   Serial.print(motorState);
  //   Serial.print(" | Deadman: ");
  //   Serial.print(deadman_active ? "ON" : "OFF");
  //   Serial.print(" | ADC: ");
  //   Serial.println(adc_throttle);
  // }
}

void writePWM(uint motorState, uint dutyCycle)
{
  uint8_t duty = (uint8_t)dutyCycle;
  uint8_t complement = 0;
  
  // Sama seperti kode sebelumnya
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

  if(motorState == 0)       // LOW A, HIGH B
    writePhases(0, duty, 0, 255, complement, 0);
  else if(motorState == 1)  // LOW A, HIGH C
    writePhases(0, 0, duty, 255, 0, complement);
  else if(motorState == 2)  // LOW B, HIGH C
    writePhases(0, 0, duty, 0, 255, complement);
  else if(motorState == 3)  // LOW B, HIGH A
    writePhases(duty, 0, 0, complement, 255, 0);
  else if(motorState == 4)  // LOW C, HIGH A
    writePhases(duty, 0, 0, complement, 0, 255);
  else if(motorState == 5)  // LOW C, HIGH B
    writePhases(0, duty, 0, 0, complement, 255);
  else                      // All off
    writePhases(0, 0, 0, 0, 0, 0);
}

void writePhases(uint ah, uint bh, uint ch, uint al, uint bl, uint cl)
{
  // Set PWM levels, low-side sudah di-invert di config
  pwm_set_both_levels(A_PWM_SLICE, ah, 255 - al);
  pwm_set_both_levels(B_PWM_SLICE, bh, 255 - bl);
  pwm_set_both_levels(C_PWM_SLICE, ch, 255 - cl);
}

uint get_halls()
{
  // Baca hall sensors dengan oversampling untuk mengurangi noise
  uint hallCounts[] = {0, 0, 0};
  
  for(uint i = 0; i < HALL_OVERSAMPLE; i++)
  {
    hallCounts[0] += gpio_get(HALL_1_PIN);
    hallCounts[1] += gpio_get(HALL_2_PIN);
    hallCounts[2] += gpio_get(HALL_3_PIN);
  }

  uint hall_result = 0;
  if (hallCounts[0] >= HALL_OVERSAMPLE / 2) hall_result |= (1<<0);
  if (hallCounts[1] >= HALL_OVERSAMPLE / 2) hall_result |= (1<<1);
  if (hallCounts[2] >= HALL_OVERSAMPLE / 2) hall_result |= (1<<2);
  
  return hall_result & 0x7;
}

//uint8_t readThrottle()
//{
  // Fungsi ini sekarang tidak digunakan karena throttle dibaca di interrupt
  // Tapi kita biarkan untuk kompatibilitas
//  return throttle_value;
//}