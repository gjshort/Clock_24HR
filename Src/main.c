/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Clock_24HR_main.c
  * @brief          : Defines all behavior for a 24-hour clock
  * @author			: Griffin Short
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim10;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM10_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define LOW 0
#define HIGH 1



// Represents, in inverted binary, the values of the LEDs
// for each value needed for display
typedef enum DISPLAY_CODE {
	ZERO  = 0b00000011,
	ONE   = 0b10011111,
	TWO   = 0b00100101,
	THREE = 0b00001101,		// a,b,c,d,e,f,g,dp
	FOUR  = 0b10011001,
	FIVE  = 0b01001001,
	SIX   = 0b01000001,
	SEVEN = 0b00011111,
	EIGHT = 0b00000001,
	NINE  = 0b00011001
}DISPLAY_CODE;

// Proper pin values for the select lines of each digit
// bits: [mux_en, mux_1, mux_0, colon_en]
typedef enum DIGIT_SEL_CODE {
	DIGIT_ONE = 0b0001, 	// Hours tens
	DIGIT_TWO = 0b0011, 	// Hours ones
	DIGIT_COLON = 0b1100,	// Colon
	DIGIT_THREE = 0b0101, 	// Minutes tens
	DIGIT_FOUR = 0b0111 	// Minutes ones
}DIGIT_SEL_CODE;

// Variables for shifting values to the shift registers
unsigned int sr_clk_state = LOW;
unsigned int sr_clk_count = 0;
unsigned int is_shifting = 0;
unsigned int shift_mask = 0b000000001;
DISPLAY_CODE shift_code = ZERO;
DIGIT_SEL_CODE sr_num = DIGIT_ONE;

// Variables for rotating through digit display
DIGIT_SEL_CODE digit_select = DIGIT_ONE;

// --------------- Translate Int to Display Code ----------------
DISPLAY_CODE int_to_disp_code(uint8_t int_val) {
	DISPLAY_CODE ret_code;
	switch(int_val) {
	case 0:
		ret_code = ZERO;
		break;
	case 1:
		ret_code = ONE;
		break;
	case 2:
		ret_code = TWO;
		break;
	case 3:
		ret_code = THREE;
		break;
	case 4:
		ret_code = FOUR;
		break;
	case 5:
		ret_code = FIVE;
		break;
	case 6:
		ret_code = SIX;
		break;
	case 7:
		ret_code = SEVEN;
		break;
	case 8:
		ret_code = EIGHT;
		break;
	case 9:
		ret_code = NINE;
		break;
	default:
		ret_code = ZERO;
		break;
	}
	return ret_code;
}
// ------------END Translate Int to Display Code ----------------


// ------------------ Shift Display Code ------------------------
// Instigates the process to shift a new digit out to a shift register
// Requires the digit number and display code.
void shift_display_code_tim(DIGIT_SEL_CODE digit, DISPLAY_CODE code) {
	shift_code = code;
	sr_num = digit;
	is_shifting = 1;
	HAL_TIM_Base_Start_IT(&htim3); // Start timer for shifting
	while(is_shifting) {} //Wait until done
}
// ----------------- END Shift Display Code ---------------------

// ------------------ Cycle Display -----------------------------
// Sets the multiplexer/select lines for the next digit to be
// displayed on the quad display. Returns the next digit in the sequence
DIGIT_SEL_CODE cycle_display(DIGIT_SEL_CODE digit) {
	switch(digit) {
	  case DIGIT_ONE:	// Tens place of hours
		  HAL_GPIO_WritePin(MUX_EN_GPIO_Port, MUX_EN_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MUX_1_GPIO_Port, MUX_1_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MUX_0_GPIO_Port, MUX_0_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Colon_EN_GPIO_Port, Colon_EN_Pin, GPIO_PIN_SET);
		  digit = DIGIT_TWO;
		  break;
	  case DIGIT_TWO:	// Ones place of hours
		  HAL_GPIO_WritePin(MUX_EN_GPIO_Port, MUX_EN_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MUX_1_GPIO_Port, MUX_1_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MUX_0_GPIO_Port, MUX_0_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Colon_EN_GPIO_Port, Colon_EN_Pin, GPIO_PIN_SET);
		  digit = DIGIT_COLON;
		  break;
	  case 0b1100:	// Colon
		  HAL_GPIO_WritePin(MUX_EN_GPIO_Port, MUX_EN_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(MUX_1_GPIO_Port, MUX_1_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(MUX_0_GPIO_Port, MUX_0_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Colon_EN_GPIO_Port, Colon_EN_Pin, GPIO_PIN_RESET);
		  digit = DIGIT_THREE;
		  break;
	  case DIGIT_THREE:	// Tens place of minutes
		  HAL_GPIO_WritePin(MUX_EN_GPIO_Port, MUX_EN_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MUX_1_GPIO_Port, MUX_1_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(MUX_0_GPIO_Port, MUX_0_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Colon_EN_GPIO_Port, Colon_EN_Pin, GPIO_PIN_SET);
		  digit = DIGIT_FOUR;
		  break;
	  case DIGIT_FOUR:	// Ones place of minutes
		  HAL_GPIO_WritePin(MUX_EN_GPIO_Port, MUX_EN_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MUX_1_GPIO_Port, MUX_1_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(MUX_0_GPIO_Port, MUX_0_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Colon_EN_GPIO_Port, Colon_EN_Pin, GPIO_PIN_SET);
		  digit = DIGIT_ONE;
		  break;
	}
	return digit;
}
// --------------------- END Cycle Display ------------------------


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
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM10_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  // Set interrupt priorities (First is shifting timer (TIM3), then RTC, then digit rotation)

  // TIM3 : Shift register clock/data timing
  //		84 MHz -> 6 Mhz with (14-1) pre-scaler and 65,535 counter
  //
  // TIM10 : Quad display rotation timing
  //		84 MHz -> 1 kHz with (42,000-1) pre-scaler and 32,768 counter

  HAL_Delay(1000);
  HAL_TIM_Base_Start_IT(&htim10); // Start timer for digit rotation

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // **************************** MAIN LOOP *********************************
  while (1)
  {


  }
  // **************************** MAIN LOOP *********************************
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x8;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY|RTC_ALARMMASK_HOURS
                              |RTC_ALARMMASK_MINUTES;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  // Starting display on clock
  shift_display_code_tim(DIGIT_ONE, int_to_disp_code((sTime.Hours & 0xF0) >> 4U));
  shift_display_code_tim(DIGIT_TWO, int_to_disp_code(sTime.Hours & 0x0F));
  shift_display_code_tim(DIGIT_THREE, int_to_disp_code((sTime.Seconds & 0xF0) >> 4U));
  shift_display_code_tim(DIGIT_FOUR, int_to_disp_code(sTime.Seconds & 0x0F));

  /* USER CODE END RTC_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 42000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 14-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SR0_Clk_Pin|SR1_Clk_Pin|SR2_Clk_Pin|SR3_Clk_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SR_Data_GPIO_Port, SR_Data_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Colon_EN_Pin|MUX_0_Pin|MUX_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MUX_EN_GPIO_Port, MUX_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SR0_Clk_Pin SR1_Clk_Pin SR2_Clk_Pin SR3_Clk_Pin */
  GPIO_InitStruct.Pin = SR0_Clk_Pin|SR1_Clk_Pin|SR2_Clk_Pin|SR3_Clk_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SR_Data_Pin */
  GPIO_InitStruct.Pin = SR_Data_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SR_Data_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Colon_EN_Pin MUX_0_Pin MUX_1_Pin */
  GPIO_InitStruct.Pin = Colon_EN_Pin|MUX_0_Pin|MUX_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MUX_EN_Pin */
  GPIO_InitStruct.Pin = MUX_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(MUX_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IN_HRS_Pin IN_MINS_Pin */
  GPIO_InitStruct.Pin = IN_HRS_Pin|IN_MINS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// ***************************** Interrupt Service Routines ****************************************

// -------------------------- Timer ISR ------------------------------
// TIM3: Shift register clocking
// Times the output shifting to the shift register indicated. Uses a
// Signals: 3 MHz square wave output is for shift register clock (cycles 0-18) on SRx_Clk_Pin
//			Shift register data being output on SR_Data_Pin. Value changes right after SRx_Clk_Pin has gone low
//
// TIM4: Display digit rotation
// Changes the display output to the next digit at 2 kHz
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// Shift register clocking timer
	if(htim == &htim3) {
		sr_clk_state = sr_clk_count % 2; // Clock cycles from 0-18
		switch(sr_num) {	// Toggle appropriate clock pin
		case DIGIT_ONE:
			HAL_GPIO_WritePin(SR0_Clk_GPIO_Port, SR0_Clk_Pin, sr_clk_state);
			break;
		case DIGIT_TWO:
			HAL_GPIO_WritePin(SR1_Clk_GPIO_Port, SR1_Clk_Pin, sr_clk_state);
			break;
		case DIGIT_THREE:
			HAL_GPIO_WritePin(SR2_Clk_GPIO_Port, SR2_Clk_Pin, sr_clk_state);
			break;
		case DIGIT_FOUR:
			HAL_GPIO_WritePin(SR3_Clk_GPIO_Port, SR3_Clk_Pin, sr_clk_state);
			break;
		default:
			break;
		}
		if(sr_clk_count == 18) { // If finished shifting, stop clock
			HAL_TIM_Base_Stop_IT(&htim3);
			sr_clk_count = 0;
			is_shifting = 0;
			shift_mask = 0b000000001; // Reset mask
			return;
		}
		else if(sr_clk_state == LOW) { // If clock has switched to LOW, update serial line
			HAL_GPIO_WritePin(SR_Data_GPIO_Port, SR_Data_Pin, shift_code & shift_mask);
			shift_mask <<= 1;
		}
		sr_clk_count++;
	}

	// Display digit timer.
	else if(htim == &htim10) {
		// Toggle pins to select digits in a left-to-right refresh order
		digit_select = cycle_display(digit_select);
	}
}
// ------------------ END Shift Register Timer ISR -----------------------------

// -------------------------- RTC ISR ------------------------------
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) {

	//HAL_TIM_Base_Stop_IT(&htim10); // Pause digit rotation

	// Get time
	RTC_TimeTypeDef time;
	RTC_DateTypeDef date;
	HAL_RTC_GetTime(hrtc, &time, FORMAT_BCD);
	HAL_RTC_GetDate(hrtc, &date, FORMAT_BCD);
	uint8_t hrs = time.Hours;
	uint8_t mins = time.Minutes;
	//uint8_t secs = time.Seconds;

	// Update display
	shift_display_code_tim(DIGIT_ONE, int_to_disp_code((hrs & 0xF0) >> 4U));
	shift_display_code_tim(DIGIT_TWO, int_to_disp_code(hrs & 0x0F));
	shift_display_code_tim(DIGIT_THREE, int_to_disp_code((mins & 0xF0) >> 4U));
	shift_display_code_tim(DIGIT_FOUR, int_to_disp_code(mins & 0x0F));

	//HAL_TIM_Base_Start_IT(&htim10); // Resume digit rotation

}
// -------------------------- END RTC ISR --------------------------

// ------------------------ Input Buttons ISR ------------------------
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn); // Debounce button

	// Get current time
	RTC_TimeTypeDef time;
	RTC_DateTypeDef date;
	HAL_RTC_GetTime(&hrtc, &time, FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &date, FORMAT_BIN);
	uint8_t hrs = time.Hours;
	uint8_t mins = time.Minutes;

	// Add an hour to RTC
	if(GPIO_Pin == IN_HRS_Pin) {
		if(hrs >= 23) {
			hrs = 0;
		}
		else {
			hrs = hrs + 1;
		}
	}
	// Add a minute to RTC
	else if(GPIO_Pin == IN_MINS_Pin) {
		if(mins >= 59) {
			mins = 0;
		}
		else {
			mins = mins + 1;
		}
	}

	// Update RTC
	time.Hours = hrs;
	time.Minutes = mins;
	time.Seconds = 0;
	if (HAL_RTC_SetTime(&hrtc, &time, FORMAT_BIN) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_RTC_SetDate(&hrtc, &date, FORMAT_BIN) != HAL_OK) {
		Error_Handler();
	}

	// Shift out new time to registers
	hrs = RTC_ByteToBcd2(hrs);
	mins = RTC_ByteToBcd2(mins);

	shift_display_code_tim(DIGIT_ONE, int_to_disp_code((hrs & 0xF0) >> 4U));
	shift_display_code_tim(DIGIT_TWO, int_to_disp_code(hrs & 0x0F));
	shift_display_code_tim(DIGIT_THREE, int_to_disp_code((mins & 0xF0) >> 4U));
	shift_display_code_tim(DIGIT_FOUR, int_to_disp_code(mins & 0x0F));

	HAL_Delay(50);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}
// -------------------- END Input Buttons ISR ------------------------

// ***************************** END Interrupt Service Routines ************************************
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

#ifdef  USE_FULL_ASSERT
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
