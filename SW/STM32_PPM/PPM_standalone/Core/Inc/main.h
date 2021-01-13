/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SN6505_EN_Pin GPIO_PIN_3
#define SN6505_EN_GPIO_Port GPIOC
#define Amp_fil_sig_int_ADC_Pin GPIO_PIN_0
#define Amp_fil_sig_int_ADC_GPIO_Port GPIOA
#define UART2_TX_RS232_Pin GPIO_PIN_2
#define UART2_TX_RS232_GPIO_Port GPIOA
#define UART2_RX_RS232_Pin GPIO_PIN_3
#define UART2_RX_RS232_GPIO_Port GPIOA
#define SPI1_NSS_for_MCU_Pin GPIO_PIN_4
#define SPI1_NSS_for_MCU_GPIO_Port GPIOA
#define SPI1_CLK_ext_ADC_Pin GPIO_PIN_5
#define SPI1_CLK_ext_ADC_GPIO_Port GPIOA
#define LT1777_SYNC_Pin GPIO_PIN_6
#define LT1777_SYNC_GPIO_Port GPIOA
#define SPI1_MOSI_ext_ADC_Pin GPIO_PIN_7
#define SPI1_MOSI_ext_ADC_GPIO_Port GPIOA
#define Gen_SP1I_NSS_MCU_Pin GPIO_PIN_9
#define Gen_SP1I_NSS_MCU_GPIO_Port GPIOE
#define Gen_SPI1_NSS_ext_ADC_Pin GPIO_PIN_11
#define Gen_SPI1_NSS_ext_ADC_GPIO_Port GPIOE
#define UART3_TX_FT230_Pin GPIO_PIN_10
#define UART3_TX_FT230_GPIO_Port GPIOB
#define UART3_RX_FT230_Pin GPIO_PIN_11
#define UART3_RX_FT230_GPIO_Port GPIOB
#define UART5_RX_Pin GPIO_PIN_12
#define UART5_RX_GPIO_Port GPIOB
#define UART5_TX_Pin GPIO_PIN_13
#define UART5_TX_GPIO_Port GPIOB
#define UART1_TX_Pin GPIO_PIN_14
#define UART1_TX_GPIO_Port GPIOB
#define UART1_RX_Pin GPIO_PIN_15
#define UART1_RX_GPIO_Port GPIOB
#define LED1_R_Pin GPIO_PIN_8
#define LED1_R_GPIO_Port GPIOD
#define LED1_G_Pin GPIO_PIN_9
#define LED1_G_GPIO_Port GPIOD
#define LED1_B_Pin GPIO_PIN_10
#define LED1_B_GPIO_Port GPIOD
#define SN6505_END11_Pin GPIO_PIN_11
#define SN6505_END11_GPIO_Port GPIOD
#define SN6505_SYNC_Pin GPIO_PIN_12
#define SN6505_SYNC_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOD
#define LED3_Pin GPIO_PIN_14
#define LED3_GPIO_Port GPIOD
#define LED4_Pin GPIO_PIN_15
#define LED4_GPIO_Port GPIOD
#define Gen_SPI1_CLK_for_ext_ADC_Pin GPIO_PIN_6
#define Gen_SPI1_CLK_for_ext_ADC_GPIO_Port GPIOC
#define BTN1_Pin GPIO_PIN_7
#define BTN1_GPIO_Port GPIOC
#define BTN2_Pin GPIO_PIN_8
#define BTN2_GPIO_Port GPIOC
#define Comp_Pin GPIO_PIN_15
#define Comp_GPIO_Port GPIOA
#define Switches_driver_enable_Pin GPIO_PIN_0
#define Switches_driver_enable_GPIO_Port GPIOD
#define S1_Pin GPIO_PIN_1
#define S1_GPIO_Port GPIOD
#define S2_Pin GPIO_PIN_2
#define S2_GPIO_Port GPIOD
#define S3_Pin GPIO_PIN_3
#define S3_GPIO_Port GPIOD
#define S4_Pin GPIO_PIN_4
#define S4_GPIO_Port GPIOD
#define S5_Pin GPIO_PIN_5
#define S5_GPIO_Port GPIOD
#define S6_Pin GPIO_PIN_6
#define S6_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
