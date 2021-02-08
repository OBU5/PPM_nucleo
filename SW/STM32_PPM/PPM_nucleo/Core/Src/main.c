/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "float.h"
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PERIOD 30
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

typedef struct States {
	uint8_t setMeasureTechnique;
	uint8_t activeMeasureTechnique;
	int16_t remainingMeasurements;
	int16_t setMeasurements;
	uint8_t measureTechniqueUpdated;
	uint8_t preparedToRunPolarizationPhase;
	uint16_t index;
} State;

State state;

union Buffer {
	uint8_t uint8[44100 * 2];
	uint16_t uint16[44100];
};
uint8_t filledBuffers = 0;
uint32_t samplesPerPeriod = 44100;
uint32_t samplesTotal = 44100 * 2;
union Buffer buffer_rx1;
union Buffer buffer_rx2;
uint8_t buffer_uart_rx[3];

uint32_t IC_Value1 = 0;
uint32_t IC_Value2 = 0;
uint32_t difference = 0;
uint32_t frequency = 0;
uint8_t firstCapturedSample = 0;  // 0- not captured, 1- captured
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//Callback for receive SPI
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_DMA_Init();
	MX_USART3_UART_Init();
	MX_USB_OTG_FS_PCD_Init();
	MX_SPI1_Init();
	MX_TIM8_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_ADC1_Init();
	MX_TIM4_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	// Start timers
	HAL_UART_Receive_IT(&huart3, buffer_uart_rx, 3);
	//HAL_TIM_Base_Start_IT(&htim3);
	/*
	 switchingCircuitIdle();
	 HAL_Delay(100);
	 runPolarizationSequence();
	 HAL_Delay(1000);
	 switchingCircuitIdle();*/

	//Configure_DMA();
	//LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */
		if (state.preparedToRunPolarizationPhase) {
			chooseActionByState();
		}
		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 216;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_TIM
			| RCC_PERIPHCLK_USART3 | RCC_PERIPHCLK_CLK48;
	PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
	PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
	PeriphClkInitStruct.TIMPresSelection = RCC_TIMPRES_ACTIVATED;

	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T4_TRGO;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_SLAVE;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
	hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 4898 - 1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM2;
	sConfigOC.Pulse = 864 + 10;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 3000;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 0xffffffff;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_IC_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 50000;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 43200 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 4898 - 1;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void) {

	/* USER CODE BEGIN TIM8_Init 0 */

	/* USER CODE END TIM8_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_SlaveConfigTypeDef sSlaveConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM8_Init 1 */

	/* USER CODE END TIM8_Init 1 */
	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 0;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = 108 - 1;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 24;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim8) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_OnePulse_Init(&htim8, TIM_OPMODE_SINGLE) != HAL_OK) {
		Error_Handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
	sSlaveConfig.InputTrigger = TIM_TS_ITR0;
	if (HAL_TIM_SlaveConfigSynchro(&htim8, &sSlaveConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM2;
	sConfigOC.Pulse = 54;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM8_Init 2 */

	/* USER CODE END TIM8_Init 2 */
	HAL_TIM_MspPostInit(&htim8);

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 2000000;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief USB_OTG_FS Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_OTG_FS_PCD_Init(void) {

	/* USER CODE BEGIN USB_OTG_FS_Init 0 */

	/* USER CODE END USB_OTG_FS_Init 0 */

	/* USER CODE BEGIN USB_OTG_FS_Init 1 */

	/* USER CODE END USB_OTG_FS_Init 1 */
	hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
	hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
	hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
	hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
	hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
	hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
	hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
	if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USB_OTG_FS_Init 2 */

	/* USER CODE END USB_OTG_FS_Init 2 */

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* Init with LL driver */
	/* DMA controller clock enable */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

	/* DMA interrupt init */
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	/* DMA2_Stream4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LD1_Pin | LD3_Pin | LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD,
			Switches_driver_enable_Pin | S1_Pin | S2_Pin | S3_Pin | S4_Pin
					| S5_Pin | S6_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : USER_Btn_Pin */
	GPIO_InitStruct.Pin = USER_Btn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
	GPIO_InitStruct.Pin = RMII_MDC_Pin | RMII_RXD0_Pin | RMII_RXD1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
	GPIO_InitStruct.Pin = LD1_Pin | LD3_Pin | LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : RMII_TXD1_Pin */
	GPIO_InitStruct.Pin = RMII_TXD1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_PowerSwitchOn_Pin */
	GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_OverCurrent_Pin */
	GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : Switches_driver_enable_Pin S1_Pin S2_Pin S3_Pin
	 S4_Pin S5_Pin S6_Pin */
	GPIO_InitStruct.Pin = Switches_driver_enable_Pin | S1_Pin | S2_Pin | S3_Pin
			| S4_Pin | S5_Pin | S6_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
	GPIO_InitStruct.Pin = RMII_TX_EN_Pin | RMII_TXD0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	//stop the ADC when in interrupt
	HAL_ADC_Stop_DMA(&hadc1);
	filledBuffers++;
	// observe interval of SPI receiving
	// Run the measurement again

	//first buffer is filled
	if (filledBuffers == 1) {

		HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &buffer_rx2.uint16,
				samplesPerPeriod);

	}

	//second buffer is filled, send data over UART
	else if (filledBuffers == 2) {

		// stop measuring
		switchingCircuitIdle();
		filledBuffers = 0;
		sendDataOverUART();

		state.remainingMeasurements--;
		//if freq should be measured only once, after the measurement, go to idle state
		if (state.remainingMeasurements == 0) {
			state.activeMeasureTechnique = 0;
			HAL_TIM_Base_Stop_IT(&htim4);

		} else {
			HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &buffer_rx1.uint16,
					samplesPerPeriod);
		}

	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		state.measureTechniqueUpdated = 1;
		prepareForNextMeasurements(buffer_uart_rx);
		char msg_buffer[18];
		//sprintf(msg_buffer, "Mode %u selected\n\r", state.setMeasureTechnique);
		HAL_UART_Transmit(&huart3, (uint8_t*) msg_buffer, strlen(msg_buffer),
				10);
		//wait for next incomming data
		HAL_UART_Receive_IT(&huart3, buffer_uart_rx, 3);
	}
}

prepareForNextMeasurements(char *receivedData) {

	//set measurement method
	switch (receivedData[0]) {
	case '0':
		state.setMeasureTechnique = 0;
		break;
	case '1':
		state.setMeasureTechnique = 1;
		break;
	case '2':
		state.setMeasureTechnique = 2;
		break;
	case '3':
		state.setMeasureTechnique = 3;
		break;
	default:
		state.setMeasureTechnique = 0;
		break;
	}

	//set number of measurements
	switch (receivedData[1]) {
	case '0':
		state.setMeasurements = -1;
		break;
	case '1':
		state.setMeasurements = 1;
		break;
	case '2':
		state.setMeasurements = 2;
		break;
	case '3':
		state.setMeasurements = 3;
		break;
	case '4':
		state.setMeasurements = 4;
		break;
	case '5':
		state.setMeasurements = 5;
		break;
	case '6':
		state.setMeasurements = 6;
		break;
	case '7':
		state.setMeasurements = 7;
		break;
	case '8':
		state.setMeasurements = 8;
		break;
	case '9':
		state.setMeasurements = 9;
		break;
	default:
		state.setMeasurements = 1;
		break;
	}
	state.preparedToRunPolarizationPhase = 1;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	filledBuffers++;

	//first buffer is filled
	if (filledBuffers == 1) {
		HAL_SPI_Receive_DMA(&hspi1, buffer_rx2.uint8, samplesPerPeriod);
	}

	//second buffer is filled, send data over UART
	else if (filledBuffers == 2) {
		// stop measuring
		switchingCircuitIdle();
		// turn off timers
		HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_2);
		HAL_TIM_PWM_Stop_IT(&htim8, TIM_CHANNEL_1);

		filledBuffers = 0;
		sendDataOverUART();

		state.remainingMeasurements--;
		//if this is the last measurement, go to idle state
		if (state.remainingMeasurements == 0) {
			state.activeMeasureTechnique = 0;
		}
		else{
			state.preparedToRunPolarizationPhase = 1;
		}

	}

	//ToModify
	//HAL_TIM_Base_Stop(&htim1);
	//HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);
	//HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_2);

	//send data over uart
	/*if (hspi == &hspi1) {
	 char msg_buffers[16];
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, SET);
	 adc = (uint16_t) 256 * buffer_rx[1] + (uint16_t) buffer_rx[2];
	 samplesPerPeriod++;
	 sprintf(msg_buffers, "%hu\n", adc);
	 HAL_UART_Transmit(&huart3, (uint8_t*) msg_buffers, strlen(msg_buffers), HAL_MAX_DELAY);

	 }*/
}

void sendDataOverUART() {
	char msg_freq[16];
	char msg_buffers[16];
	uint16_t adc = 0;
	int i = 0;
	if (state.activeMeasureTechnique == 1
			|| state.activeMeasureTechnique == 2) {
		// first buffer
		for (i = 0; i < samplesPerPeriod; i++) {
			adc = (uint16_t) (buffer_rx1.uint8[i])
					+ (uint16_t) (256 * buffer_rx1.uint8[i + 1]);
			sprintf(msg_buffers, "%hu\n", adc);
			HAL_UART_Transmit(&huart3, (uint8_t*) msg_buffers,
					strlen(msg_buffers),
					HAL_MAX_DELAY);
			i++;
		}
		//second buffer
		for (i = 0; i < samplesPerPeriod; i++) {
			adc = (uint16_t) (buffer_rx2.uint8[i])
					+ (uint16_t) (256 * buffer_rx2.uint8[i + 1]);
			sprintf(msg_buffers, "%hu\n", adc);
			HAL_UART_Transmit(&huart3, (uint8_t*) msg_buffers,
					strlen(msg_buffers),
					HAL_MAX_DELAY);
			i++;
		}
		sprintf(msg_buffers, ";%hu\n", 50);
		HAL_UART_Transmit(&huart3, (uint8_t*) msg_buffers, strlen(msg_buffers),
		HAL_MAX_DELAY);
	} else if (state.activeMeasureTechnique == 3) {
		//send frequency
		sprintf(msg_freq, "%d\n", frequency);
		HAL_UART_Transmit(&huart3, (uint8_t*) msg_freq, strlen(msg_freq),
		HAL_MAX_DELAY);
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM4) {
		//HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
	}

	// period  100 ms
	if (htim->Instance == TIM3) {/*
		if (state.index < PERIOD) {
			chooseActionByState();
		} else {
			state.index = 0;
		}
		state.index++;*/

	}

}

void Configure_DMA(void) {
	/* (1) Enable the clock of DMA2 */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
	/* (2) Configure the DMA functionnal parameters */
	/* Configuration of the DMA parameters can be done using unitary functions or using the specific configure function */
	/* Unitary Functions */

	LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_0,
	LL_DMA_DIRECTION_MEMORY_TO_MEMORY);
	LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_0, LL_DMA_PRIORITY_HIGH);
	LL_DMA_SetMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MODE_NORMAL);
	LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_PERIPH_INCREMENT);
	LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_HALFWORD);
	LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_HALFWORD);

	/* Configure Function */
//  LL_DMA_ConfigTransfer(DMA2, LL_DMA_STREAM_0, LL_DMA_DIRECTION_MEMORY_TO_MEMORY |
//                                               LL_DMA_PRIORITY_HIGH              |
//                                               LL_DMA_MODE_NORMAL                |
//                                               LL_DMA_PERIPH_INCREMENT           |
//                                               LL_DMA_MEMORY_INCREMENT           |
//                                               LL_DMA_PDATAALIGN_WORD            |
//                                               LL_DMA_MDATAALIGN_WORD);
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_0, samplesPerPeriod);
	LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_0,
			(uint32_t) &hspi1.Instance->DR, (uint32_t) &buffer_rx1,
			LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_STREAM_0));

	/* (3) Configure NVIC for DMA transfer complete/error interrupts */
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_0);
	LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_0);
	NVIC_SetPriority(DMA2_Stream0_IRQn, 0);
	NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		measureFrequencyWithTimer(htim);
	}
}

void initState() {
	state.setMeasureTechnique = 0;
	state.activeMeasureTechnique = 0;
	state.remainingMeasurements = 0;
	state.preparedToRunPolarizationPhase = 0;
	state.index = 0;
}

void chooseActionByState() {
	// if new measurement technique was set, update remaining measurements as well
	if (state.measureTechniqueUpdated) {
		state.remainingMeasurements = state.setMeasurements;
		state.activeMeasureTechnique = state.setMeasureTechnique;
		state.measureTechniqueUpdated = 0;

	}
	switch (state.activeMeasureTechnique) {

	case 0:
		// Idle state
		showOnLEDs(0, 0, 0);
		break;
	case 1:
		// measure with external ADC
		runPolarizationSequence();
		measureWithExternalADC();
		break;
	case 2:
		// measure with internal ADC
		runPolarizationSequence();
		measureWithInternalADC();
		break;
	case 3:
		// measure with comparator
		runPolarizationSequence();
		measureWithComparator();
		break;

	}
}

void showOnLEDs(uint8_t L1, uint8_t L2, uint8_t L3) {

	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, L1);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, L2);
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, L3);
}

void measureWithExternalADC() {
	// visualise
	showOnLEDs(1, 0, 0);
	// Start SPI communication over DMA
	HAL_SPI_Receive_DMA(&hspi1, buffer_rx1.uint8, samplesPerPeriod);
	//turn on timers
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);	// SPI -  MCU NSS
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2); 	// SPI -  External ADC NSS
	HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_1);	// SPI -  CLK
}

void measureWithInternalADC() {
	// visualise
	showOnLEDs(0, 1, 0);
	//start ADC
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &buffer_rx1.uint16, samplesPerPeriod);
	// start timer
	HAL_TIM_Base_Start_IT(&htim4);
}

void measureWithComparator() {
	// visualise
	showOnLEDs(0, 0, 1);
	// run the timer
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
}

//mode = 1 ... run only once, mode = 0 ... run infinity times
void measureFrequencyWithTimer(TIM_HandleTypeDef *htim) {
	if (firstCapturedSample == 0) {
		IC_Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		firstCapturedSample = 1;
	}

	else if (firstCapturedSample) {
		IC_Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		difference = IC_Value2 - IC_Value1;
		frequency = /*HAL_RCC_GetHCLKFreq() /*/difference;
		firstCapturedSample = 0;
		sendDataOverUART();

		state.remainingMeasurements--;
		//if freq should be measured only once, after the measurement, go to idle state
		if (state.remainingMeasurements == 0) {
			state.activeMeasureTechnique = 0;
		}
		else{
			state.preparedToRunPolarizationPhase = 1;
		}
	}
}

void runPolarizationSequence() {
	//polarization phase will be ready after measurements
	state.preparedToRunPolarizationPhase = 0;

	// visualise
	showOnLEDs(1, 1, 1);
	//run sequnece T2 - prepare for polarization
	HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 1);
	HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 0);
	HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, 1);
	HAL_GPIO_WritePin(S4_GPIO_Port, S4_Pin, 0);
	HAL_GPIO_WritePin(S5_GPIO_Port, S5_Pin, 0);
	HAL_GPIO_WritePin(S6_GPIO_Port, S6_Pin, 1);
	HAL_Delay(5);

	//run sequnece T3 - Polarization phase
	HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 1);
	HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 0);
	HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, 1);
	HAL_GPIO_WritePin(S4_GPIO_Port, S4_Pin, 1);
	HAL_GPIO_WritePin(S5_GPIO_Port, S5_Pin, 0);
	HAL_GPIO_WritePin(S6_GPIO_Port, S6_Pin, 1);
	HAL_Delay(1000);

	//run sequnece T4 - Coil discharge
	HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 0);
	HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 0);
	HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, 1);
	HAL_GPIO_WritePin(S4_GPIO_Port, S4_Pin, 0);
	HAL_GPIO_WritePin(S5_GPIO_Port, S5_Pin, 1);
	HAL_GPIO_WritePin(S6_GPIO_Port, S6_Pin, 1);
	HAL_Delay(10);

	//run sequnece T5 - wait before measuring
	HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 1);
	HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 0);
	HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, 1);
	HAL_GPIO_WritePin(S4_GPIO_Port, S4_Pin, 1);
	HAL_GPIO_WritePin(S5_GPIO_Port, S5_Pin, 0);
	HAL_GPIO_WritePin(S6_GPIO_Port, S6_Pin, 0);
	HAL_Delay(5);

	//run sequnece T6 - measure
	HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 1);
	HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 1);
	HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, 0);
	HAL_GPIO_WritePin(S4_GPIO_Port, S4_Pin, 1);
	HAL_GPIO_WritePin(S5_GPIO_Port, S5_Pin, 0);
	HAL_GPIO_WritePin(S6_GPIO_Port, S6_Pin, 1);

}

void switchingCircuitIdle() {
	// visualise
	showOnLEDs(0, 0, 0);
	// active low output enable
	HAL_GPIO_WritePin(Switches_driver_enable_GPIO_Port,
	Switches_driver_enable_Pin, 0);
	//also run the sequnece T1 (isolate amplifier)
	HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 0);
	HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 0);
	HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, 1);
	HAL_GPIO_WritePin(S4_GPIO_Port, S4_Pin, 0);
	HAL_GPIO_WritePin(S5_GPIO_Port, S5_Pin, 0);
	HAL_GPIO_WritePin(S6_GPIO_Port, S6_Pin, 0);
}

void switchingCircuitOff() {
	// active low output enable
	HAL_GPIO_WritePin(Switches_driver_enable_GPIO_Port,
	Switches_driver_enable_Pin, 1);
	HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 0);
	HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 0);
	HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, 0);
	HAL_GPIO_WritePin(S4_GPIO_Port, S4_Pin, 0);
	HAL_GPIO_WritePin(S5_GPIO_Port, S5_Pin, 0);
	HAL_GPIO_WritePin(S6_GPIO_Port, S6_Pin, 0);
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
