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
#define TIMER  TIM4

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi4;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;
DMA_HandleTypeDef hdma_tim2_ch1;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

typedef struct States {
	uint8_t extAdcReadyToSend;
	uint8_t intAdcReadyToSend;
	uint8_t compReadyToSend;

	uint8_t extAdcActiveState;
	uint8_t intAdcActiveState;
	uint8_t compActiveState;

	uint8_t extAdcMeasuring;
	uint8_t intAdcMeasuring;
	uint8_t compMeasuring;

	uint8_t extAdcSetState;
	uint8_t intAdcSetState;
	uint8_t compSetState;

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
uint8_t filledBuffersExtADC = 0;
uint8_t filledBuffersIntADC = 0;
uint32_t samplesPerPeriod = 44100;
uint32_t samplesTotal = 44100 * 2;

union Buffer buffer_extAdc_1;
union Buffer buffer_extAdc_2;
union Buffer buffer_intAdc_1;
union Buffer buffer_intAdc_2;

uint32_t buffer_comp[4001];
uint8_t receivedChars[50];
uint8_t receivedCharIndex;
uint8_t buffer_uart_rx[1];

uint32_t IC_Value1 = 0;
uint32_t IC_Value2 = 0;
uint32_t difference = 0;
uint32_t frequency = 0;
uint8_t firstCapturedSample = 0;  // 0- not captured, 1- captured

uint32_t polarizationTime = 5000; // 5 seconds
uint32_t timeIndex = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI4_Init(void);
static void MX_TIM5_Init(void);
static void MX_UART7_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	MX_ADC1_Init();
	MX_SPI1_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM8_Init();
	MX_UART5_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_I2C1_Init();
	MX_I2C3_Init();
	MX_SPI4_Init();
	MX_TIM5_Init();
	MX_UART7_Init();
	MX_TIM6_Init();
	/* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart3, buffer_uart_rx, 1);
	HAL_TIM_Base_Start_IT(&htim5);
	char msg_buffers[25];
	uint16_t index = 0;
	switchingCircuitIdle();
	// visualise
	set_LED1(0, 0, 0);
	HAL_Delay(1000);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		parseText();

		// if new measurement technique was updated - change state regarding to the update
		if (state.measureTechniqueUpdated && !state.extAdcMeasuring && !state.intAdcMeasuring && !state.compMeasuring && !state.extAdcReadyToSend && !state.intAdcReadyToSend
				&& !state.compReadyToSend) {
			updateState();
		}
		if (state.preparedToRunPolarizationPhase && !state.extAdcMeasuring && !state.intAdcMeasuring && !state.compMeasuring && !state.extAdcReadyToSend && !state.intAdcReadyToSend
				&& !state.compReadyToSend) {
			runMeasurementMethod();
		}
		//idle state - if no method is active and there are no data to be sent
		else if (!state.extAdcActiveState && !state.intAdcActiveState && !state.compActiveState && !state.extAdcReadyToSend && !state.intAdcReadyToSend && !state.compReadyToSend) {
			set_LED1(0, 0, 0);
		} else {
			sendMeasuredData();
		}

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
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 5;
	RCC_OscInitStruct.PLL.PLLN = 216;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
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
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_TIM | RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_USART3 | RCC_PERIPHCLK_UART5 | RCC_PERIPHCLK_UART7 | RCC_PERIPHCLK_I2C1
			| RCC_PERIPHCLK_I2C3;
	PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	PeriphClkInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
	PeriphClkInitStruct.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
	PeriphClkInitStruct.Uart7ClockSelection = RCC_UART7CLKSOURCE_PCLK1;
	PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
	PeriphClkInitStruct.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
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
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T6_TRGO;
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
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x20404768;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief I2C3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C3_Init(void) {

	/* USER CODE BEGIN I2C3_Init 0 */

	/* USER CODE END I2C3_Init 0 */

	/* USER CODE BEGIN I2C3_Init 1 */

	/* USER CODE END I2C3_Init 1 */
	hi2c3.Instance = I2C3;
	hi2c3.Init.Timing = 0x20404768;
	hi2c3.Init.OwnAddress1 = 0;
	hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c3.Init.OwnAddress2 = 0;
	hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c3) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C3_Init 2 */

	/* USER CODE END I2C3_Init 2 */

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
 * @brief SPI4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI4_Init(void) {

	/* USER CODE BEGIN SPI4_Init 0 */

	/* USER CODE END SPI4_Init 0 */

	/* USER CODE BEGIN SPI4_Init 1 */

	/* USER CODE END SPI4_Init 1 */
	/* SPI4 parameter configuration*/
	hspi4.Instance = SPI4;
	hspi4.Init.Mode = SPI_MODE_MASTER;
	hspi4.Init.Direction = SPI_DIRECTION_2LINES;
	hspi4.Init.DataSize = SPI_DATASIZE_4BIT;
	hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi4.Init.NSS = SPI_NSS_HARD_OUTPUT;
	hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi4.Init.CRCPolynomial = 7;
	hspi4.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi4) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI4_Init 2 */

	/* USER CODE END SPI4_Init 2 */

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
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM2;
	sConfigOC.Pulse = 864 + 10;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 3000;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
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
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) {
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
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
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

	TIM_SlaveConfigTypeDef sSlaveConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 0;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
	sSlaveConfig.InputTrigger = TIM_TS_ITR0;
	if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 0;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim4);

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void) {

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 0;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 2160 - 1;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim5) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */

}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void) {

	/* USER CODE BEGIN TIM6_Init 0 */

	/* USER CODE END TIM6_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 0;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 4898 - 1;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM6_Init 2 */

	/* USER CODE END TIM6_Init 2 */

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
	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM2;
	sConfigOC.Pulse = 54;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
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
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM8_Init 2 */

	/* USER CODE END TIM8_Init 2 */
	HAL_TIM_MspPostInit(&htim8);

}

/**
 * @brief UART5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART5_Init(void) {

	/* USER CODE BEGIN UART5_Init 0 */

	/* USER CODE END UART5_Init 0 */

	/* USER CODE BEGIN UART5_Init 1 */

	/* USER CODE END UART5_Init 1 */
	huart5.Instance = UART5;
	huart5.Init.BaudRate = 115200;
	huart5.Init.WordLength = UART_WORDLENGTH_8B;
	huart5.Init.StopBits = UART_STOPBITS_1;
	huart5.Init.Parity = UART_PARITY_NONE;
	huart5.Init.Mode = UART_MODE_TX_RX;
	huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart5.Init.OverSampling = UART_OVERSAMPLING_16;
	huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart5) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN UART5_Init 2 */

	/* USER CODE END UART5_Init 2 */

}

/**
 * @brief UART7 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART7_Init(void) {

	/* USER CODE BEGIN UART7_Init 0 */

	/* USER CODE END UART7_Init 0 */

	/* USER CODE BEGIN UART7_Init 1 */

	/* USER CODE END UART7_Init 1 */
	huart7.Instance = UART7;
	huart7.Init.BaudRate = 115200;
	huart7.Init.WordLength = UART_WORDLENGTH_8B;
	huart7.Init.StopBits = UART_STOPBITS_1;
	huart7.Init.Parity = UART_PARITY_NONE;
	huart7.Init.Mode = UART_MODE_TX_RX;
	huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart7.Init.OverSampling = UART_OVERSAMPLING_16;
	huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart7) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN UART7_Init 2 */

	/* USER CODE END UART7_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

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
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	/* DMA2_Stream4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
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
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SN6505_EN_GPIO_Port, SN6505_EN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD,
			LED1_R_Pin | LED1_G_Pin | LED1_B_Pin | SN6505_END11_Pin | LED2_Pin | LED3_Pin | LED4_Pin | Switches_driver_enable_Pin | S1_Pin | S2_Pin | S3_Pin | S4_Pin | S5_Pin | S6_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : SN6505_EN_Pin */
	GPIO_InitStruct.Pin = SN6505_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SN6505_EN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LED1_R_Pin LED1_G_Pin LED1_B_Pin SN6505_END11_Pin
	 LED2_Pin LED3_Pin LED4_Pin Switches_driver_enable_Pin
	 S1_Pin S2_Pin S3_Pin S4_Pin
	 S5_Pin S6_Pin */
	GPIO_InitStruct.Pin = LED1_R_Pin | LED1_G_Pin | LED1_B_Pin | SN6505_END11_Pin | LED2_Pin | LED3_Pin | LED4_Pin | Switches_driver_enable_Pin | S1_Pin | S2_Pin | S3_Pin | S4_Pin | S5_Pin | S6_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : BTN1_Pin BTN2_Pin */
	GPIO_InitStruct.Pin = BTN1_Pin | BTN2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void runMeasurementMethod() {

	//if measurement method is set, run the polarization sequence
	if ((state.remainingMeasurements > 0) || (state.remainingMeasurements == -1) && ((state.extAdcActiveState == 1) || (state.intAdcActiveState == 1) || (state.compActiveState == 1))) {
		runPolarizationSequence();

		if (state.intAdcActiveState == 1) {
			measureWithInternalADC();
		}
		if (state.extAdcActiveState == 1) {
			measureWithExternalADC();
		}
		if (state.compActiveState == 1) {
			measureWithComparator();
		}
	}
}

void runPolarizationSequence() {

	//polarization phase will be ready after measurements
	state.preparedToRunPolarizationPhase = 0;
	// visualise
	set_LED1(1, 1, 1);
	//run sequnece T2 - prepare for polarization
	HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 1);
	HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 0);
	HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, 1);
	HAL_GPIO_WritePin(S4_GPIO_Port, S4_Pin, 0);
	HAL_GPIO_WritePin(S5_GPIO_Port, S5_Pin, 0);
	HAL_GPIO_WritePin(S6_GPIO_Port, S6_Pin, 1);
	delay_ms(5);

	//run sequnece T3 - Polarization phase
	HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 1);
	HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 0);
	HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, 1);
	HAL_GPIO_WritePin(S4_GPIO_Port, S4_Pin, 1);
	HAL_GPIO_WritePin(S5_GPIO_Port, S5_Pin, 0);
	HAL_GPIO_WritePin(S6_GPIO_Port, S6_Pin, 1);
	delay_ms(polarizationTime);

	//run sequnece T4 - Coil discharge
	HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 0);
	HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 0);
	HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, 1);
	HAL_GPIO_WritePin(S4_GPIO_Port, S4_Pin, 0);
	HAL_GPIO_WritePin(S5_GPIO_Port, S5_Pin, 0);
	HAL_GPIO_WritePin(S6_GPIO_Port, S6_Pin, 1);
	delay_us(200);

	//run sequnece T5 - Coil discharge
	HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 0);
	HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 0);
	HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, 1);
	HAL_GPIO_WritePin(S4_GPIO_Port, S4_Pin, 0);
	HAL_GPIO_WritePin(S5_GPIO_Port, S5_Pin, 1);
	HAL_GPIO_WritePin(S6_GPIO_Port, S6_Pin, 1);
	delay_ms(10);

	//run sequnece T6 - wait before measuring
	HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 0);
	HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 0);
	HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, 1);
	HAL_GPIO_WritePin(S4_GPIO_Port, S4_Pin, 0);
	HAL_GPIO_WritePin(S5_GPIO_Port, S5_Pin, 1);
	HAL_GPIO_WritePin(S6_GPIO_Port, S6_Pin, 0);
	delay_ms(5);
	//run sequnece T7 - measure
	HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 0);
	HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 1);
	HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, 0);
	HAL_GPIO_WritePin(S4_GPIO_Port, S4_Pin, 0);
	HAL_GPIO_WritePin(S5_GPIO_Port, S5_Pin, 1);
	HAL_GPIO_WritePin(S6_GPIO_Port, S6_Pin, 0);

}

void set_LED1(uint8_t R, uint8_t G, uint8_t B) {
	HAL_GPIO_WritePin(LED1_R_GPIO_Port, LED1_R_Pin, G);
	HAL_GPIO_WritePin(LED1_G_GPIO_Port, LED1_G_Pin, B);
	HAL_GPIO_WritePin(LED1_B_GPIO_Port, LED1_B_Pin, R);
}
void set_LED2(uint8_t val) {
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, val);
}

void set_LED3(uint8_t val) {
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, val);
}

void set_LED4(uint8_t val) {
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, val);
}

void switchingCircuitIdle() {
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM5) {
		if (timeIndex > 0) {
			timeIndex -= 10;
		}
	}

	if (htim->Instance == TIM6) {
	}

}

void delay_us(uint32_t delay_us) {
	timeIndex = delay_us;
	while (timeIndex > 0)
		;
}

void delay_ms(uint32_t delay_us) {
	timeIndex = delay_us * 1000;
	while (timeIndex > 0)
		;
}

void measureWithExternalADC() {
	// visualise
	set_LED1(0, 1, 0);
	// Start SPI communication over DMA
	HAL_SPI_Receive_DMA(&hspi1, buffer_extAdc_1.uint8, samplesPerPeriod);
	//turn on timers
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);	// SPI -  MCU NSS
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2); 	// SPI -  External ADC NSS
	HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_1);	// SPI -  CLK
	set_LED2(1);
	state.extAdcMeasuring = 1;
}

void measureWithInternalADC() {
	// visualise
	set_LED1(0, 1, 0);
	//start ADC
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &buffer_intAdc_1.uint16, samplesPerPeriod);
	// start timer
	HAL_TIM_Base_Start_IT(&htim6);
	set_LED3(1);
	state.intAdcMeasuring = 1;
}

void measureWithComparator() {
	// visualise
	set_LED1(0, 1, 0);
	// run the timer
	//HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_1, buffer_comp, 4001);
	set_LED4(1);
	state.compMeasuring = 1;

}

//mode = 1 ... run only once, mode = 0 ... run infinity times
void measureFrequencyWithTimer(TIM_HandleTypeDef *htim) {
	HAL_TIM_IC_Stop_DMA(&htim2, TIM_CHANNEL_1);
	set_LED4(0);
	state.compReadyToSend = 1;
	state.compMeasuring = 0;

	//only if all measurements were done
	if ((state.extAdcMeasuring == 0) && (state.intAdcMeasuring == 0) && (state.compMeasuring == 0)) {
		switchingCircuitIdle();
		// -1 indicates infinity measurements
		if (state.remainingMeasurements != -1) {
			state.remainingMeasurements--;
		}
	}
	//if freq should be measured only once, after the measurement, go to idle state
	if (state.remainingMeasurements == 0) {
		state.compActiveState = 0;
		state.compSetState = 0;
	} else {
		if ((state.extAdcMeasuring == 0) && (state.intAdcMeasuring == 0) && (state.compMeasuring == 0)) {
			state.preparedToRunPolarizationPhase = 1;
		}
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		measureFrequencyWithTimer(htim);
	}
}

void initState() {
	state.extAdcReadyToSend = 0;
	state.intAdcReadyToSend = 0;
	state.compReadyToSend = 0;

	state.extAdcActiveState = 0;
	state.extAdcSetState = 0;
	state.intAdcActiveState = 0;

	state.intAdcSetState = 0;
	state.compActiveState = 0;
	state.compSetState = 0;

	state.setMeasurements = 0;
	state.remainingMeasurements = 0;
	state.preparedToRunPolarizationPhase = 0;
	state.index = 0;
}

void sendDataOverUART() {
	char msg_freq[16];
	char msg_buffers[16];
	uint16_t adc = 0;
	int i = 0;
	if (state.extAdcActiveState == 1) {
		// first buffer
		for (i = 0; i < samplesPerPeriod; i++) {
			adc = (buffer_extAdc_1.uint16[i]);
			sprintf(msg_buffers, "%hu\n", adc);
			HAL_UART_Transmit(&huart3, (uint8_t*) msg_buffers, strlen(msg_buffers),
			HAL_MAX_DELAY);
		}
		//second buffer
		for (i = 0; i < samplesPerPeriod; i++) {
			adc = (buffer_extAdc_2.uint16[i]);
			sprintf(msg_buffers, "%hu\n", adc);
			HAL_UART_Transmit(&huart3, (uint8_t*) msg_buffers, strlen(msg_buffers),
			HAL_MAX_DELAY);
		}
		sprintf(msg_buffers, ";%hu\n", 50);
		HAL_UART_Transmit(&huart3, (uint8_t*) msg_buffers, strlen(msg_buffers),
		HAL_MAX_DELAY);
	} else if (state.intAdcActiveState == 1) {
		// first buffer
		for (i = 0; i < samplesPerPeriod; i++) {
			adc = (buffer_intAdc_1.uint16[i]);
			sprintf(msg_buffers, "%hu\n", adc);
			HAL_UART_Transmit(&huart3, (uint8_t*) msg_buffers, strlen(msg_buffers),
			HAL_MAX_DELAY);
		}
		//second buffer
		for (i = 0; i < samplesPerPeriod; i++) {
			adc = (buffer_intAdc_2.uint16[i]);
			sprintf(msg_buffers, "%hu\n", adc);
			HAL_UART_Transmit(&huart3, (uint8_t*) msg_buffers, strlen(msg_buffers),
			HAL_MAX_DELAY);
		}
		sprintf(msg_buffers, ";%hu\n", 50);
		HAL_UART_Transmit(&huart3, (uint8_t*) msg_buffers, strlen(msg_buffers),
		HAL_MAX_DELAY);
	} else if (state.compActiveState == 1) {
		//send frequency
		for (i = 0; i < 3999; i++) {
			uint32_t freq = buffer_comp[i + 1] - buffer_comp[i];
			sprintf(msg_freq, "%d\n", freq);
			HAL_UART_Transmit(&huart3, (uint8_t*) msg_freq, strlen(msg_freq), HAL_MAX_DELAY);
		}
	}

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	//stop the ADC when in interrupt
	ADC1->CR2 &= ~ADC_CR2_DMA;
	// if function HAL_ADC_Stop_DMA(&hadc1) would be called, it wouldn't be possible to Start DMA again.

	filledBuffersIntADC++;

	//first buffer is filled
	if (filledBuffersIntADC == 1) {
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &buffer_intAdc_2.uint16, samplesPerPeriod);
	}

	//second buffer is filled, send data over UART
	else if (filledBuffersIntADC == 2) {
		// stop measuring

		set_LED3(0);
		state.intAdcReadyToSend = 1;
		state.intAdcMeasuring = 0;
		// turn off timers
		HAL_TIM_Base_Stop_IT(&htim6);

		filledBuffersIntADC = 0;
		//only if all measurements were done
		if ((state.extAdcMeasuring == 0) && (state.intAdcMeasuring == 0) && (state.compMeasuring == 0)) {
			switchingCircuitIdle();
			// -1 indicates infinity measurements
			if (state.remainingMeasurements != -1) {
				state.remainingMeasurements--;
			}
		}
		//if this is the last measurement, go to idle state
		if (state.remainingMeasurements == 0) {
			state.intAdcActiveState = 0;
			state.intAdcSetState = 0;
		} else {
			if ((state.extAdcMeasuring == 0) && (state.intAdcMeasuring == 0) && (state.compMeasuring == 0)) {
				state.preparedToRunPolarizationPhase = 1;
			}
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		HAL_UART_Receive_IT(&huart3, buffer_uart_rx, 1);
		state.measureTechniqueUpdated = 1;
		//prepareForNextMeasurements(buffer_uart_rx);
		receivedChars[(receivedCharIndex++) % 50] = buffer_uart_rx[0];
	}
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	filledBuffersExtADC++;

	// turn off timers
	HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop_IT(&htim8, TIM_CHANNEL_1);
	SPI1->CR2 &= ~SPI_CR2_RXDMAEN;

	//first buffer is filled
	if (filledBuffersExtADC == 1) {

		HAL_SPI_Receive_DMA(&hspi1, buffer_extAdc_2.uint8, samplesPerPeriod);

		HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_1);
	}

	//second buffer is filled, send data over UART
	else if (filledBuffersExtADC == 2) {
		// stop measuring
		set_LED2(0);
		state.extAdcReadyToSend = 1;
		state.extAdcMeasuring = 0;

		filledBuffersExtADC = 0;

		//only if all measurements were done
		if ((state.extAdcMeasuring == 0) && (state.intAdcMeasuring == 0) && (state.compMeasuring == 0)) {
			switchingCircuitIdle();
			// -1 indicates infinity measurements
			if (state.remainingMeasurements != -1) {
				state.remainingMeasurements--;
			}
		}
		//if this is the last measurement, go to idle state
		if (state.remainingMeasurements == 0) {
			state.extAdcActiveState = 0;
			state.extAdcSetState = 0;
		} else {
			if ((state.extAdcMeasuring == 0) && (state.intAdcMeasuring == 0) && (state.compMeasuring == 0)) {
				state.preparedToRunPolarizationPhase = 1;
			}
		}
	}
}

int parseText() {
	//check if there is two times character * indicating complete command
	uint8_t i, indexOfFirstSpecialChar, indexOfSecondSpecialChar, specialCharCount = 0;
	char msg_buffers[80];
	char receivedCommand[50];

	for (i = 0; i < strlen(receivedCommand); i++) {
		receivedCommand[i] = '\0';
	}
	for (i = 0; i < strlen(msg_buffers); i++) {
		msg_buffers[i] = '\0';
	}
	for (i = 0; i < strlen(receivedChars); i++) {
		if (receivedChars[i] == '*') {
			if (specialCharCount == 0) {
				indexOfFirstSpecialChar = i;
			} else if (specialCharCount == 1) {
				indexOfSecondSpecialChar = i;
			}
			specialCharCount++;
		}
	}
	if (specialCharCount == 1) {
		set_LED1(1, 0, 0);
		return 0;
	} else if (specialCharCount == 2) {
		sprintf(msg_buffers, "New state was set\n");
		HAL_UART_Transmit(&huart3, (uint8_t*) msg_buffers, strlen(msg_buffers), HAL_MAX_DELAY);
	} else if (specialCharCount > 2) {
		set_LED1(1, 0, 0);
		clearReceivedCharsBuffer(); // receivedChars needs to be cleared
	} else {
		return 0;
	}

	// get string between special chars
	strncpy(receivedCommand, receivedChars + indexOfFirstSpecialChar + 1, indexOfSecondSpecialChar - indexOfFirstSpecialChar - 1);
	receivedCommand[indexOfSecondSpecialChar - indexOfFirstSpecialChar - 1] = '\0';

	//if specialCharCount == 2
	// Extract the first token - command

	char *command = strtok(receivedCommand, ":");
	char *method = strtok(NULL, ":");
	char *count = strtok(NULL, ":");

	//*IDN*
	if (strcmp(command, "IDN") == 0) {
		sprintf(msg_buffers, "This is proton precession magnetometer, ver. 1\n");
		HAL_UART_Transmit(&huart3, (uint8_t*) msg_buffers, strlen(msg_buffers),
		HAL_MAX_DELAY);
	}
	//*SET:parameter:value*
	if (strcmp(command, "SET") == 0) {
		//polarization time
		if (strcmp(method, "polT") == 0) {
			polarizationTime = atoi(count);
			if (count < 5000) {
				polarizationTime = 5000;
			} else if (count > 20000) {
				polarizationTime = 20000;
			}
		}
	}
	//*MEAS:method:count*
	else if (strcmp(command, "MEAS") == 0) {
		//external ADC only
		if (strcmp(method, "extADC") == 0) {
			state.extAdcSetState = 1;
			state.intAdcSetState = 0;
			state.compSetState = 0;
			state.measureTechniqueUpdated = 1;
		}
		//internal ADC only
		else if (strcmp(method, "intADC") == 0) {
			state.extAdcSetState = 0;
			state.intAdcSetState = 1;
			state.compSetState = 0;
			state.measureTechniqueUpdated = 1;
		}
		//comparator only
		else if (strcmp(method, "comp") == 0) {
			state.extAdcSetState = 0;
			state.intAdcSetState = 0;
			state.compSetState = 1;
			state.measureTechniqueUpdated = 1;
		}
		//external ADC + internal ADC
		else if ((strcmp(method, "extADC+intADC")) == 0 || (strcmp(method, "intADC+extADC")) == 0) {
			state.extAdcSetState = 1;
			state.intAdcSetState = 1;
			state.compSetState = 0;
			state.measureTechniqueUpdated = 1;
		}
		//external ADC + comparator
		else if ((strcmp(method, "extADC+comp") == 0) || (strcmp(method, "comp+extADC") == 0)) {
			state.extAdcSetState = 1;
			state.intAdcSetState = 0;
			state.compSetState = 1;
			state.measureTechniqueUpdated = 1;
		}
		//internal ADC + comparator
		else if ((strcmp(method, "intADC+comp") == 0) || (strcmp(method, "comp+intADC")) == 0) {
			state.extAdcSetState = 0;
			state.intAdcSetState = 1;
			state.compSetState = 1;
			state.measureTechniqueUpdated = 1;
		}
		//external ADC + internal ADC + comparator
		else if ((strcmp(method, "extADC+intADC+comp") == 0) || (strcmp(method, "intADC+extADC+comp") == 0)) {
			state.compSetState = 1;
			state.extAdcSetState = 1;
			state.intAdcSetState = 1;
			state.measureTechniqueUpdated = 1;
		} else /* default: */
		{

		}
		if (strcmp(count, "INF") == 0) {
			state.setMeasurements = -1;
		} else if (strcmp(count, "") == 0) {
			state.setMeasurements = 1;
		} else {
			state.setMeasurements = atoi(count);
		}

	}
	/* more else if clauses */
	else /* default: */
	{
	}
	clearReceivedCharsBuffer();
}

void clearReceivedCharsBuffer() {
	int i = 0;
	receivedCharIndex = 0;
	for (i = 0; i < strlen(receivedChars); i++) {
		receivedChars[i] = '\0';
	}
}

void sendMeasuredData() {
	char msg_freq[16];
	char msg_buffers[16];
	uint16_t adc = 0;
	int i = 0;
	if ((state.extAdcReadyToSend == 1)) {
		set_LED1(0, 0, 1);
		sprintf(msg_buffers, "*extADC:\n");
		HAL_UART_Transmit(&huart3, (uint8_t*) msg_buffers, strlen(msg_buffers),
		HAL_MAX_DELAY);
		// first buffer
		for (i = 0; i < samplesPerPeriod; i++) {
			adc = (buffer_extAdc_1.uint16[i]);
			sprintf(msg_buffers, "%hu\n", adc);
			HAL_UART_Transmit(&huart3, (uint8_t*) msg_buffers, strlen(msg_buffers),
			HAL_MAX_DELAY);
		}
		//second buffer
		for (i = 0; i < samplesPerPeriod; i++) {
			adc = (buffer_extAdc_2.uint16[i]);
			sprintf(msg_buffers, "%hu\n", adc);
			HAL_UART_Transmit(&huart3, (uint8_t*) msg_buffers, strlen(msg_buffers),
			HAL_MAX_DELAY);
		}
		sprintf(msg_buffers, "*\n");
		HAL_UART_Transmit(&huart3, (uint8_t*) msg_buffers, strlen(msg_buffers),
		HAL_MAX_DELAY);
		state.extAdcReadyToSend = 0;
	}

	if ((state.intAdcReadyToSend == 1)) {
		set_LED1(0, 0, 1);
		sprintf(msg_buffers, "*intADC:\n");
		HAL_UART_Transmit(&huart3, (uint8_t*) msg_buffers, strlen(msg_buffers),
		HAL_MAX_DELAY);
		// first buffer
		for (i = 0; i < samplesPerPeriod; i++) {
			adc = (buffer_intAdc_1.uint16[i]);
			sprintf(msg_buffers, "%hu\n", adc);
			HAL_UART_Transmit(&huart3, (uint8_t*) msg_buffers, strlen(msg_buffers),
			HAL_MAX_DELAY);
		}
		//second buffer
		for (i = 0; i < samplesPerPeriod; i++) {
			adc = (buffer_intAdc_2.uint16[i]);
			sprintf(msg_buffers, "%hu\n", adc);
			HAL_UART_Transmit(&huart3, (uint8_t*) msg_buffers, strlen(msg_buffers),
			HAL_MAX_DELAY);
		}
		sprintf(msg_buffers, "*\n");
		HAL_UART_Transmit(&huart3, (uint8_t*) msg_buffers, strlen(msg_buffers),
		HAL_MAX_DELAY);
		state.intAdcReadyToSend = 0;
	}

	if ((state.compReadyToSend == 1)) {
		set_LED1(0, 0, 1);
		//send frequency
		sprintf(msg_buffers, "*comp:\n");
		HAL_UART_Transmit(&huart3, (uint8_t*) msg_buffers, strlen(msg_buffers),
		HAL_MAX_DELAY);
		for (i = 0; i < 4000; i++) {
			uint32_t freq = buffer_comp[i + 1] - buffer_comp[i];
			sprintf(msg_freq, "%d\n", freq);
			HAL_UART_Transmit(&huart3, (uint8_t*) msg_freq, strlen(msg_freq), HAL_MAX_DELAY);
		}
		sprintf(msg_buffers, "*\n");
		HAL_UART_Transmit(&huart3, (uint8_t*) msg_buffers, strlen(msg_buffers),
		HAL_MAX_DELAY);
		set_LED1(0, 0, 0);
		state.compReadyToSend = 0;
	}
}

void updateState() {
	state.remainingMeasurements = state.setMeasurements;
	state.extAdcActiveState = state.extAdcSetState;
	state.intAdcActiveState = state.intAdcSetState;
	state.compActiveState = state.compSetState;
	state.preparedToRunPolarizationPhase = 1;
	state.measureTechniqueUpdated = 0;
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
