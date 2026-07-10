/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "arm_math.h" // The CMSIS-DSP hardware acceleration library
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Constants
//#define PI 3.14159265359f
//#define POLE_PAIRS 8.0f // From your motor config
#define MAX_VOLTAGE 27.7f
#define KP 0.05f
#define KI 0.0005f

#define SQRT3_OVER_2 0.86602540378f
#define PWM_PERIOD 4199.0f // TIM1 ARR value

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

// State variables
volatile bool hardware_fault_latched = false;
volatile float32_t current_vbus_v = 48.0f; // Initialized to nominal
volatile float32_t system_max_voltage = 27.7f;

volatile float current_velocity_rad_s = 0.0f;
volatile float base_electrical_angle = 0.0f;
volatile uint32_t last_hall_time_us = 0;
volatile float32_t previous_base_angle = 0.0f;

volatile float32_t offset_a = 2048.0f;
volatile float32_t offset_c = 2048.0f;

// Global variables to monitor in STM32CubeIDE Live Expressions (Optional, for debugging)
volatile float32_t I_alpha, I_beta;
volatile float32_t I_d, I_q;

// Standard Hall to Electrical Angle Lookup Table (in radians)
// Index is the 3-bit Hall state (0-7, where 0 and 7 are invalid hardware errors)
const float hall_angle_table[8] = {
    0.0f,               // 0: Invalid
    (5.0f * PI) / 3.0f, // 1: 300 degrees
    (1.0f * PI) / 3.0f, // 2: 60 degrees
    0.0f,               // 3: 0 degrees
    (3.0f * PI) / 3.0f, // 4: 180 degrees
    (4.0f * PI) / 3.0f, // 5: 240 degrees
    (2.0f * PI) / 3.0f, // 6: 120 degrees
    0.0f                // 7: Invalid
};


typedef struct {
    float32_t Kp;
    float32_t Ki;
    float32_t integral_sum;
    float32_t max_output;
} PI_Controller;

// Initialize the D and Q controllers
// (Note: Kp and Ki usually require tuning based on motor inductance and resistance.
// We will start with safe, low baseline values).

PI_Controller pi_d = { .Kp = KP, .Ki = KI, .integral_sum = 0.0f, .max_output = MAX_VOLTAGE };
PI_Controller pi_q = { .Kp = KP, .Ki = KI, .integral_sum = 0.0f, .max_output = MAX_VOLTAGE };

// Target commands (In Amperes)
volatile float32_t target_Id = 0.0f;  // Always 0 for maximum efficiency
volatile float32_t target_Iq = 0.0f;  // This will eventually be linked to your throttle ADC

// Output Voltage Vectors
volatile float32_t V_d, V_q;
volatile float32_t V_alpha, V_beta;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN EV */
extern TIM_HandleTypeDef htim2;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles ADC1, ADC2 and ADC3 global interrupts.
  */
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  HAL_ADC_IRQHandler(&hadc2);
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  HAL_GPIO_EXTI_IRQHandler(DRV_FAULT_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/* USER CODE BEGIN 1 */
// Fast PI execution function
float32_t run_PI_Controller(PI_Controller *pi, float32_t target, float32_t measured) {
    float32_t error = target - measured;

    // Proportional term
    float32_t proportional = pi->Kp * error;

    // Integral term (Sum of errors over time)
    pi->integral_sum += pi->Ki * error;

    // Anti-windup limit for the integral
    if (pi->integral_sum > pi->max_output) pi->integral_sum = pi->max_output;
    if (pi->integral_sum < -pi->max_output) pi->integral_sum = -pi->max_output;

    // Calculate total output voltage
    float32_t output = proportional + pi->integral_sum;

    // Final Output Clamping
    if (output > pi->max_output) output = pi->max_output;
    if (output < -pi->max_output) output = -pi->max_output;

    return output;
}

// Safely resets accumulated integral limits after a hardware fault
void Reset_FOC_Integrators(void)
{
    pi_d.integral_sum = 0.0f;
    pi_q.integral_sum = 0.0f;
}

// Helper functions for Min/Max
float32_t fmaxf32(float32_t a, float32_t b) { return (a > b) ? a : b; }
float32_t fminf32(float32_t a, float32_t b) { return (a < b) ? a : b; }

void execute_SVPWM(float32_t v_alpha, float32_t v_beta)
{
    // 1. Pseudo-Inverse Clarke (Stationary 2-phase to 3-phase)
    float32_t v_a = v_alpha;
    float32_t v_b = -0.5f * v_alpha + SQRT3_OVER_2 * v_beta;
    float32_t v_c = -0.5f * v_alpha - SQRT3_OVER_2 * v_beta;

    // 2. Find the Min and Max of the three phases
    float32_t v_max = fmaxf32(v_a, fmaxf32(v_b, v_c));
    float32_t v_min = fminf32(v_a, fminf32(v_b, v_c));

    // 3. Calculate the Zero-Sequence (Common-Mode) voltage to inject
    float32_t v_center = -(v_max + v_min) / 2.0f;

    // PREVENT NaN EXPLOSION: Clamp Vbus to a minimum of 10 Volts
	float32_t safe_vbus = current_vbus_v;
	if (safe_vbus < 10.0f) safe_vbus = 10.0f;

	// 4. Inject the saddle profile and calculate duty cycle
	float32_t duty_a = ((v_a + v_center) / safe_vbus) + 0.5f;
	float32_t duty_b = ((v_b + v_center) / safe_vbus) + 0.5f;
	float32_t duty_c = ((v_c + v_center) / safe_vbus) + 0.5f;

    // 5. Clamp the duty cycles to prevent ADC noise collisions
	if (duty_a > 0.95f) duty_a = 0.95f;
	else if (duty_a < 0.05f) duty_a = 0.05f;

	if (duty_b > 0.95f) duty_b = 0.95f;
	else if (duty_b < 0.05f) duty_b = 0.05f;

	if (duty_c > 0.95f) duty_c = 0.95f;
	else if (duty_c < 0.05f) duty_c = 0.05f;

    // 6. Convert Duty Cycle to Timer Ticks and load directly into TIM1 Registers
    TIM1->CCR1 = (uint32_t)(duty_a * PWM_PERIOD); // Phase A
    TIM1->CCR2 = (uint32_t)(duty_b * PWM_PERIOD); // Phase B
    TIM1->CCR3 = (uint32_t)(duty_c * PWM_PERIOD); // Phase C
}

// This function is automatically called by the hardware the moment the ADCs
// finish reading Phase A and Phase C at the center of the PWM OFF-time.
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
    	// ========================================================
		// 1. READ RAW ADC & RECONSTRUCT CURRENT (AMPERES)
		// ========================================================
		// Read the raw 12-bit ADC values (0 - 4095)
		uint16_t raw_a = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
		uint16_t raw_c = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);

		// Convert to Amperes: (ADC - Offset) * Multiplier
		float32_t I_a = ((float32_t)raw_a - offset_a) * 0.080566f;
		float32_t I_c = ((float32_t)raw_c - offset_c) * 0.080566f;

		// Calculate Phase B using Kirchhoff's Current Law
		float32_t I_b = -I_a - I_c;
        // ========================================================
		// 2. ROTOR ANGLE ESTIMATION
		// ========================================================
		// Calculate how much time has passed since the last physical Hall click
		uint32_t current_time_us = __HAL_TIM_GET_COUNTER(&htim2);
		uint32_t time_since_last_hall = current_time_us - last_hall_time_us;

		// If the motor is stalled or moving extremely slowly, zero the velocity to prevent wild math
		if (time_since_last_hall > 100000) { // 100ms timeout
			current_velocity_rad_s = 0.0f;
		}

		// Interpolate the high-resolution angle: Angle = Base + (Velocity * Time)
		float interpolated_angle = base_electrical_angle +
								   (current_velocity_rad_s * ((float)time_since_last_hall * 0.000001f));

		// Wrap the angle to keep it strictly between 0 and 2*PI
		while (interpolated_angle >= (2.0f * PI)) interpolated_angle -= (2.0f * PI);
		while (interpolated_angle < 0.0f) interpolated_angle += (2.0f * PI);


		// ========================================================
		// 3. FOC TRANSFORMS (CLARKE & PARK)
		// ========================================================

		// Clarke Transform: 3-phase (a, b) to 2-phase stationary (alpha, beta)
		// Note: arm_clarke_f32 automatically handles the math under the hood.
		arm_clarke_f32(I_a, I_b, (float32_t *)&I_alpha, (float32_t *)&I_beta);

		// Calculate fast hardware sine and cosine for the Park transform
		float32_t sin_theta = arm_sin_f32(interpolated_angle);
		float32_t cos_theta = arm_cos_f32(interpolated_angle);

		// Park Transform: Stationary (alpha, beta) to Rotating (d, q)
		arm_park_f32(I_alpha, I_beta, (float32_t *)&I_d, (float32_t *)&I_q, sin_theta, cos_theta);
		// ========================================================
		// 4. PI CONTROLLERS (Current to Voltage)
		// ========================================================

		// 1. Calculate D-Axis voltage first (limit it to the absolute system max)
		pi_d.max_output = system_max_voltage;
		V_d = run_PI_Controller(&pi_d, target_Id, I_d);

		// 2. Calculate remaining voltage headroom for the Q-Axis
		float32_t v_d_squared = V_d * V_d;
		float32_t max_voltage_squared = system_max_voltage * system_max_voltage;
		float32_t math_diff = max_voltage_squared - v_d_squared;
		// PREVENT NaN EXPLOSION: Clamp negative floating-point errors to 0
		if (math_diff < 0.0f) math_diff = 0.0f;

		float32_t q_max_headroom = 0.0f;

		// Use hardware-accelerated square root
		arm_sqrt_f32(math_diff, &q_max_headroom);

		// 3. Constrain Q-Axis to perfectly ride the edge of the voltage circle
		pi_q.max_output = q_max_headroom;
		V_q = run_PI_Controller(&pi_q, target_Iq, I_q);

		// ========================================================
		// 5. INVERSE PARK TRANSFORM
		// ========================================================
		// Now that we have the required DC voltages (V_d, V_q), we must convert
		// them back into the stationary AC frame (V_alpha, V_beta) to feed the MOSFETs.

		arm_inv_park_f32(V_d, V_q, (float32_t *)&V_alpha, (float32_t *)&V_beta, sin_theta, cos_theta);
		// ========================================================
		// 6. SPACE VECTOR PWM (SVPWM)
		// ========================================================
		// This calculates the duty cycles and physically updates the MOSFET timer registers!
		execute_SVPWM(V_alpha, V_beta);
    }
}

// Dedicated function to read the initial rotor angle at boot
void Init_Rotor_Angle(void)
{
    uint8_t hall_A = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
    uint8_t hall_B = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);
    uint8_t hall_C = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11);

    uint8_t hall_state = (hall_C << 2) | (hall_B << 1) | hall_A;

    // CRITICAL: Ignore hardware faults (all LOW or all HIGH)
    if (hall_state == 0 || hall_state == 7) return;

    base_electrical_angle = hall_angle_table[hall_state];
    previous_base_angle = base_electrical_angle;
}

// This fires on ANY rising or falling edge of PB6, PB7, or PC11
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// ========================================================
	// 1. EMERGENCY HARDWARE FAULT (Microsecond Shutdown)
	// ========================================================
	if (GPIO_Pin == GPIO_PIN_12) // DRV_FAULT (PC12)
	{
		// Double-check the pin is actually LOW (filters out noise)
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == GPIO_PIN_RESET)
		{
			// 1. Clear the Main Output Enable (MOE) bit.
			// This instantly forces all TIM1 PWM pins to a safe 0V state without stopping the timer.
			TIM1->BDTR &= ~TIM_BDTR_MOE;

			// 2. Disable the DRV8302 gate driver
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); // EN_GATE

			// 3. Latch the fault so main.c handles the 2-second recovery
			hardware_fault_latched = true;
		}
		return; // Exit immediately
	}

	// hall sensor logic
    if (GPIO_Pin == GPIO_PIN_6 || GPIO_Pin == GPIO_PIN_7 || GPIO_Pin == GPIO_PIN_11)
    {
        uint32_t current_time_us = __HAL_TIM_GET_COUNTER(&htim2);
        uint32_t delta_t = current_time_us - last_hall_time_us;

        // ONLY process state updates if it is real physical movement (Debounce)
        if (delta_t > 50)
        {
            // 1. Read the pins
            uint8_t hall_A = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
            uint8_t hall_B = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);
            uint8_t hall_C = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11);

            uint8_t hall_state = (hall_C << 2) | (hall_B << 1) | hall_A;

            if (hall_state == 0 || hall_state == 7) return; // ignore invalid/glitch codes

            // 3. Update time and Angle
            last_hall_time_us = current_time_us;
            base_electrical_angle = hall_angle_table[hall_state];

            // 4. Calculate Directional Velocity
            float32_t delta_angle = base_electrical_angle - previous_base_angle;

            // Handle boundary wrap
            if (delta_angle < -PI) delta_angle += (2.0f * PI);
            else if (delta_angle > PI) delta_angle -= (2.0f * PI);

            current_velocity_rad_s = delta_angle / ((float32_t)delta_t * 0.000001f);

            // 5. Move previous angle update INSIDE the valid block
            previous_base_angle = base_electrical_angle;
        }
    }
}
/* USER CODE END 1 */
