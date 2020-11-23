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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ETH_HandleTypeDef heth;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
uint8_t filledBuffers = 0;
uint32_t  samplesPerPeriod = 44100;
uint32_t samplesTotal = 44100 * 2;
uint8_t buffer_rx1[44100 * 2];
uint8_t buffer_rx2[44100 * 2];

uint32_t IC_Value1 = 0;
uint32_t IC_Value2 = 0;
uint32_t difference = 0;
float frequency = 0;
uint8_t firstCapturedSample = 0;  // 0- not captured, 1- captured
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
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
	MX_ETH_Init();
	MX_USART3_UART_Init();
	MX_USB_OTG_FS_PCD_Init();
	MX_SPI1_Init();
	MX_TIM8_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */

	// Start timers
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);

	// Start SPI communication over DMA
	HAL_SPI_Receive_DMA(&hspi1, buffer_rx1, samplesPerPeriod);
	//Configure_DMA();
	//LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

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
 * @brief ETH Initialization Function
 * @param None
 * @retval None
 */
static void MX_ETH_Init(void) {

	/* USER CODE BEGIN ETH_Init 0 */

	/* USER CODE END ETH_Init 0 */

	/* USER CODE BEGIN ETH_Init 1 */

	/* USER CODE END ETH_Init 1 */
	heth.Instance = ETH;
	heth.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
	heth.Init.PhyAddress = LAN8742A_PHY_ADDRESS;
	heth.Init.MACAddr[0] = 0x00;
	heth.Init.MACAddr[1] = 0x80;
	heth.Init.MACAddr[2] = 0xE1;
	heth.Init.MACAddr[3] = 0x00;
	heth.Init.MACAddr[4] = 0x00;
	heth.Init.MACAddr[5] = 0x00;
	heth.Init.RxMode = ETH_RXPOLLING_MODE;
	heth.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
	heth.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;

	/* USER CODE BEGIN MACADDRESS */

	/* USER CODE END MACADDRESS */

	if (HAL_ETH_Init(&heth) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ETH_Init 2 */

	/* USER CODE END ETH_Init 2 */

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
	huart3.Init.BaudRate = 115200;
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
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LD1_Pin | LD3_Pin | LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : USER_Btn_Pin */
	GPIO_InitStruct.Pin = USER_Btn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PA4 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
	GPIO_InitStruct.Pin = LD1_Pin | LD3_Pin | LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

}

/* USER CODE BEGIN 4 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	filledBuffers++;

	// observe interval of SPI receiving
	HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
	// Run the measurement again

	//first buffer is filled
	if (filledBuffers == 1) {
		HAL_SPI_Receive_DMA(&hspi1, buffer_rx2, samplesPerPeriod);
	}

	//second buffer is filled, send data over UART
	else if (filledBuffers == 2) {
		// turn off timers
		HAL_TIM_Base_Stop(&htim1);
		HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_2);
		HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);

		filledBuffers = 0;
		sendDataOverUART();


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
	//send frequency
	sprintf(msg_freq, "%d\n", frequency);
	//HAL_UART_Transmit(&huart3, (uint8_t*) msg_freq, strlen(msg_freq), HAL_MAX_DELAY);

	// first buffer
	for (i = 0; i < samplesPerPeriod; i++) {
		adc = (uint16_t) (buffer_rx1[i]) + (uint16_t) (256*buffer_rx1[i+1]);
		sprintf(msg_buffers, "%hu\n", adc);
		HAL_UART_Transmit(&huart3, (uint8_t*) msg_buffers, strlen(msg_buffers), HAL_MAX_DELAY);
		i++;
	}
	//second buffer
	for (i = 0; i < samplesPerPeriod; i++) {
		adc = (uint16_t) (buffer_rx2[i]) + (uint16_t) (256*buffer_rx2[i+1]);
		sprintf(msg_buffers, "%hu\n", adc);
		HAL_UART_Transmit(&huart3, (uint8_t*) msg_buffers, strlen(msg_buffers), HAL_MAX_DELAY);
		i++;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

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
		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
		if (firstCapturedSample == 0) {
			IC_Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			firstCapturedSample = 1;
		}

		else if (firstCapturedSample) {
			IC_Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			difference = IC_Value2 - IC_Value1;
			frequency = HAL_RCC_GetHCLKFreq() / difference;
			firstCapturedSample = 0;
		}
	}
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
