#include <Arduino.h>
#include <SimpleFOC.h>
#include "board_config.h"
#include "motor_config.h"
#include "drv8302.h"
#include "helpers.h"

// System clock initialization sequence tailored to VESC 4's 8MHz crystal architecture
extern "C" void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) { Error_Handler(); }
}

void jumpToSystemBootloader();

DRV8302 drv(PIN_EN_GATE, PIN_DCCAL, PIN_DRV_FAULT);

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);
BLDCDriver6PWM driver = BLDCDriver6PWM(PWM_AH, PWM_AL, PWM_BH, PWM_BL, PWM_CH, PWM_CL, EN_GATE);

HallSensor sensor = HallSensor(PIN_HALL_A, PIN_HALL_B, PIN_HALL_C, MOTOR_POLE_PAIRS);
LowsideCurrentSense current_sense = LowsideCurrentSense(CURRENT_SHUNT_RES, CURRENT_AMP_GAIN, PIN_CURR_A, _NC, PIN_CURR_C);

// Global flags configuration profiles
float target_current = 0.0f;
bool deadman_bypass = true;
bool drv_fault_bypass = true; // Set to true for automated high-speed latch resetting

void doA() { sensor.handleA(); }
void doB() { sensor.handleB(); }
void doC() { sensor.handleC(); }

void setup() {
    Serial.begin(115200);
    SimpleFOCDebug::enable(&Serial);
    
    pinMode(PIN_DEADMAN, INPUT); 
    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(PIN_LED_GREEN, OUTPUT);
    
    digitalWrite(PIN_LED_RED, HIGH); 

    // ============================ POWER
    uint32_t timeout = millis();
    while (!Serial && (millis() - timeout < 3000)) { delay(10); }
    Serial.println("USB Serial connected. Initializing hardware...");

    analogReadResolution(12);
    driver.voltage_power_supply = readBatteryVoltage();
    driver.voltage_limit = DRIVER_VOLTAGE_LIMIT;
    Serial.print("Detected Power Supply Voltage: ");
    Serial.println(readBatteryVoltage());

    Serial.println("Waiting for High Voltage power supply...");
    while (readBatteryVoltage() < 8.0f) {
        digitalWrite(PIN_LED_RED, !digitalRead(PIN_LED_RED)); 
        delay(250); 
    }

    Serial.println("SUCCESS: High Voltage detected.");
    digitalWrite(PIN_LED_RED, LOW);

    // ============================

    drv.init();
    drv.disableGate(); 
    delay(50);
    drv.enableGate();
    delay(100);

    if (drv.isFaulted()) {
        Serial.println("CRITICAL: Power is ON, but DRV is throwing a hard fault at boot. Halting.");
        digitalWrite(PIN_LED_RED, HIGH);
        while(1) { delay(1000); }
    }
    
    sensor.pullup = Pullup::USE_EXTERN; 
    sensor.init();
    sensor.enableInterrupts(doA, doB, doC);
    motor.linkSensor(&sensor);

    driver.pwm_frequency = FOC_PWM_FREQ; 
    driver.dead_zone = 0.05f; 
    
    if (!driver.init()) {
        Serial.println("CRITICAL: Driver 6-PWM failed to initialize!");
        digitalWrite(PIN_LED_RED, HIGH);
        while(1) { delay(100); }
    }
    motor.linkDriver(&driver);

    // --- AUTOMATED VESC DC CURRENT AMPLIFIER CALIBRATION ---
    Serial.println("Enabling DRV8302 internal DC amplifier calibration...");
    digitalWrite(PIN_DCCAL, HIGH); 
    delay(50);                    

    current_sense.linkDriver(&driver);
    if (!current_sense.init()) {
        Serial.println("CRITICAL: Low-side current sense failed!");
        digitalWrite(PIN_DCCAL, LOW); 
        digitalWrite(PIN_LED_RED, HIGH);
        while(1) { delay(100); }
    }
    
    digitalWrite(PIN_DCCAL, LOW);  
    delay(10);                     
    motor.linkCurrentSense(&current_sense);

    // --- FOC PARAMS SETUP ---
    motor.controller = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::foc_current;
    motor.current_limit = MOTOR_CURRENT_LIMIT;
    motor.voltage_sensor_align = 1.4f; // Optimal alignment setting for 12V 3A tracking
    
    // Low-resistance motor PID tuned parameters
    motor.PID_current_q.P = 0.015f; 
    motor.PID_current_d.P = 0.015f;
    motor.PID_current_q.I = 1.0f;   
    motor.PID_current_d.I = 1.0f;
    motor.PID_current_q.output_ramp = 1000.0f; 
    motor.PID_current_d.output_ramp = 1000.0f;
    
    motor.monitor_downsample = 100;
    motor.useMonitoring(Serial);
    
    motor.init();
    
    Serial.println("Starting FOC Alignment...");
    motor.initFOC(); 

    Serial.println("Setup complete. Ready for commands.");
}

void loop() {
    static uint32_t fault_start_time = 0;

    if (drv.isFaulted()) {
        if (fault_start_time == 0) {
            fault_start_time = millis();
        } 
        else if (millis() - fault_start_time > 10) { 
            if (drv_fault_bypass) {
                // High-speed auto-recovery clearing loop
                drv.disableGate();
                delayMicroseconds(5); 
                drv.enableGate();
                fault_start_time = 0; 
                Serial.println("BYPASS NOTICE: Transient noise or gate spike cleared from DRV. Resuming loop.");
            } 
            else {
                digitalWrite(PIN_LED_RED, HIGH); 
                drv.disableGate();
                motor.move(0.0f); 
                
                if (readBatteryVoltage() < 10.0f) {
                    Serial.println("Power loss detected. Going to sleep...");
                    while (readBatteryVoltage() < 10.0f) {
                        digitalWrite(PIN_LED_RED, !digitalRead(PIN_LED_RED));
                        delay(500);
                    }
                    Serial.println("Power restored. Rebooting STM32...");
                    delay(100);
                    NVIC_SystemReset();
                }

                Serial.println("DRV SUSTAINED FAULT DETECTED - Attempting recovery...");
                delay(500); 
                drv.enableGate();
                delay(10);
                
                if (drv.isFaulted()) {
                    Serial.println("CRITICAL: Fault persists. Halting.");
                    digitalWrite(PIN_LED_RED, HIGH);
                    while (1) { delay(1000); } 
                } else {
                    digitalWrite(PIN_LED_RED, LOW); 
                    Serial.println("Recovery successful. Resuming normal operation.");
                    fault_start_time = 0;
                }
            }
        }
    } 
    else {
        fault_start_time = 0;
    }

    if (!deadmanPressed() && !deadman_bypass) {
        target_current = 0.0f;
        motor.move(0); 
        motor.loopFOC(); 
        return; 
    }

    static int loop_count = 0;
    if (loop_count++ > 100) {
        driver.voltage_power_supply = readBatteryVoltage();
        loop_count = 0;
    }

    // Intercept serial instruction tokens before numerical parsers
    if (Serial.available() > 0) {
        char peek_char = Serial.peek();
        if ((peek_char >= 'a' && peek_char <= 'z') || (peek_char >= 'A' && peek_char <= 'Z')) {
            String cmd = Serial.readStringUntil('\n');
            cmd.trim();
            if (cmd.equalsIgnoreCase("DFU") || cmd.equalsIgnoreCase("RESET")) {
                drv.disableGate(); 
                jumpToSystemBootloader();
            }
        }
    }

    readSerialTarget();

    if (abs(target_current) > 0.05f) {
        digitalWrite(PIN_LED_GREEN, HIGH);
    } else {
        if ((millis() / 500) % 2 == 0) {
            digitalWrite(PIN_LED_GREEN, HIGH);
        } else {
            digitalWrite(PIN_LED_GREEN, LOW);
        }
    }

    motor.loopFOC();
    motor.move(target_current);
}

// FLASH MENGGUNAKAN USB BELUM BERHASIL DIPAKE
void jumpToSystemBootloader() {
    Serial.println("Executing Software Jump to Native USB DFU Mode...");
    delay(100);
    
    __disable_irq();
    
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    for (int i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    __HAL_RCC_SYSCFG_CLK_ENABLE();
    SYSCFG->MEMRMP = 0x01; 

    uint32_t dfu_address = 0x1FFF0000;
    uint32_t jump_address = *(__IO uint32_t*)(dfu_address + 4);
    void (*JumpToApplication)(void) = (void (*)(void))jump_address;
    
    __set_MSP(*(__IO uint32_t*)dfu_address);
    JumpToApplication();
    while(1); 
}