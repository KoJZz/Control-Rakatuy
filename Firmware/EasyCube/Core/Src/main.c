/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
volatile uint8_t usb_throttle_target = 0;
const uint8_t hallToMotor[8] = {255, 1, 3, 2, 5, 6, 4, 255};

uint32_t lastDebugPrint = 0;
uint32_t faultResetCount = 0;
float v_bat = 0.0f;
uint16_t raw_throttle = 0;

// Variables to catch the hardware-synced ADC readings
volatile uint16_t current_phase_A = 0;
volatile uint16_t current_phase_C = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void Commutate(uint8_t state, uint16_t throttle);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// Retarget standard library printf to the USB CDC interface safely
int _write(int file, char *ptr, int len) {
    uint32_t timeout = 500000; // Failsafe timeout to prevent locking the MCU

    // Wait until the USB interface is no longer BUSY
    while (CDC_Transmit_FS((uint8_t*)ptr, len) == USBD_BUSY && timeout-- > 0) {
        // Do nothing, just wait
    }
    return len;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  // Start the High-Side PWM channels
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // Phase C
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // Phase B
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // Phase A

  // Start the Low-Side Complementary channels
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  Commutate(0, 0);
  // 2. Start the Hardware-Synced Current Sensing
  HAL_ADCEx_InjectedStart_IT(&hadc1);

  setvbuf(stdout, NULL, _IONBF, 0); // Disable printf buffering
  // 3. Calibrate ADC & Check High Voltage
  printf("Waiting for High Voltage power supply...\r\n");
  do {
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 5);
	HAL_ADC_GetValue(&hadc1);             // <-- ADD THIS: Physically pulls data to clear EOC flag
	HAL_ADC_PollForConversion(&hadc1, 5);
	uint16_t raw_v = HAL_ADC_GetValue(&hadc1);
	v_bat = (raw_v / 4095.0f) * 3.3f * ((39000.0f + 2200.0f) / 2200.0f);
	HAL_Delay(100);
  } while (v_bat < 8.0f);

  printf("SUCCESS: High Voltage detected at %.2fV\r\n", v_bat);

  // 4. Enable the DRV8302 Gate Driver
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET); // EN_GATE HIGH
  HAL_Delay(100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // --- Kickstart & Instant-Stop Logic ---
	  static uint8_t active_throttle = 0;

	  if (usb_throttle_target > 0 && active_throttle == 0) {
		  // Kickstart: The motor is stopped. Read halls manually to fire the first commutation.
		  uint8_t hallA = (GPIOB->IDR & GPIO_PIN_6)  ? 1 : 0;
		  uint8_t hallB = (GPIOB->IDR & GPIO_PIN_7)  ? 1 : 0;
		  uint8_t hallC = (GPIOC->IDR & GPIO_PIN_11) ? 1 : 0;
		  uint8_t hall_val = hallA | (hallB << 1) | (hallC << 2);

		  Commutate(hallToMotor[hall_val], usb_throttle_target);
	  }
	  else if (usb_throttle_target == 0 && active_throttle > 0) {
		  // Instant Stop: Cut power immediately instead of waiting for a slow coasting interrupt
		  Commutate(0, 0);
	  }

	  active_throttle = usb_throttle_target; // Update the tracker
	  // ===== 1. Hardware Fault Check (DRV8302 Active-Low Fault) =====
	  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == GPIO_PIN_RESET) {
		  faultResetCount++;

		  // --- ADD THESE TWO LINES FOR SAFETY ---
		  usb_throttle_target = 0; // Force user to manually throttle up again
		  Commutate(0, 0);         // Instantly disconnect all MOSFETs

		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET); // LED_RED ON
		  printf("!!! DRV FAULT !!! Resetting EN_GATE. Throttle Zeroed.\r\n");

		  // Pulse EN_GATE low to clear latched faults
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
		  HAL_Delay(1); // 1 ms guarantees a successful hardware latch clear
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
	  } else {
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); // LED_RED OFF
	  }

	  // ===== 2. Read Regular ADC (Throttle & Voltage) =====
	  HAL_ADC_Start(&hadc1);
	  if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK) {
		  raw_throttle = HAL_ADC_GetValue(&hadc1); // Rank 1: Throttle
	  }
	  if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK) {
		  uint16_t raw_v = HAL_ADC_GetValue(&hadc1); // Rank 2: Bus Voltage
		  v_bat = (raw_v / 4095.0f) * 3.3f * ((39000.0f + 2200.0f) / 2200.0f);
	  }

	  // Turn on Green LED if throttle is active (using your USB override variable)
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, (usb_throttle_target > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);

	  // ===== 3. Telemetry Heartbeat (Non-blocking, 500ms) =====
	  if (HAL_GetTick() - lastDebugPrint >= 500) {
		  lastDebugPrint = HAL_GetTick();

		  printf("[DATA] Thr_Target: %d | IA_Raw: %d | IC_Raw: %d | Bus: %.2fV | Resets: %lu\r\n",
				  usb_throttle_target,
				  current_phase_A,
				  current_phase_C,
				  v_bat,
				  faultResetCount);
	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_8;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 2;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_9;
  sConfigInjected.InjectedRank = 2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 4199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 1;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 67;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Commutate(uint8_t state, uint16_t throttle) {
	// SAFETY OVERRIDE: If throttle is 0, force state to 0 to coast/freewheel
	if (throttle == 0) {
		state = 0;
	}

	uint32_t duty = (throttle * 4199) / 255;

    // 1. Force ALL gates OFF instantly to prevent any cross-conduction during state changes
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE |
                    TIM_CCER_CC2E | TIM_CCER_CC2NE |
                    TIM_CCER_CC3E | TIM_CCER_CC3NE);

    // 2. Set the duty cycle for all channels (doesn't output yet because CCER is off)
    TIM1->CCR1 = duty; // Phase C
    TIM1->CCR2 = duty; // Phase B
    TIM1->CCR3 = duty; // Phase A

    // 3. Enable only the specific High and Low sides needed for this step
    switch (state) {
        case 1:
            // Phase C High, Phase A Low. Phase B floats.
            TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC3NE);
            // Set Phase A duty to 0 so the Low side is fully ON (Complementary logic)
            TIM1->CCR3 = 0;
            break;
        case 2:
            // Phase C High, Phase B Low. Phase A floats.
            TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2NE);
            TIM1->CCR2 = 0;
            break;
        case 3:
            // Phase A High, Phase B Low. Phase C floats.
            TIM1->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC2NE);
            TIM1->CCR2 = 0;
            break;
        case 4:
            // Phase A High, Phase C Low. Phase B floats.
            TIM1->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC1NE);
            TIM1->CCR1 = 0;
            break;
        case 5:
            // Phase B High, Phase C Low. Phase A floats.
            TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC1NE);
            TIM1->CCR1 = 0;
            break;
        case 6:
            // Phase B High, Phase A Low. Phase C floats.
            TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC3NE);
            TIM1->CCR3 = 0;
            break;
        default:
            // State 0 or invalid: All gates remain OFF (Coasting/Freewheeling)
            break;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // A static variable retains its value between interrupt fires
    static uint8_t last_hall_val = 255;

    if(GPIO_Pin == GPIO_PIN_6 || GPIO_Pin == GPIO_PIN_7 || GPIO_Pin == GPIO_PIN_11)
    {
        uint8_t hallA = (GPIOB->IDR & GPIO_PIN_6)  ? 1 : 0;
        uint8_t hallB = (GPIOB->IDR & GPIO_PIN_7)  ? 1 : 0;
        uint8_t hallC = (GPIOC->IDR & GPIO_PIN_11) ? 1 : 0;

        uint8_t hall_val = hallA | (hallB << 1) | (hallC << 2);

        // ONLY commutate if the hall state actually changed (ignores hardware bounce)
        if (hall_val != last_hall_val) {
            last_hall_val = hall_val; // Update the latch
            uint8_t state = hallToMotor[hall_val];
            Commutate(state, usb_throttle_target);
        }
    }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if(hadc->Instance == ADC1) {
        // Grab the data out of the hardware registers.
        // Rank 1 is Phase C (PB0), Rank 2 is Phase A (PB1) as set in CubeMX.
        current_phase_C = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
        current_phase_A = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
