/*
 * gpio.c
 *
 *  Created on: Apr 30, 2021
 *      Author: OBU
 */

#include "gpio.h"



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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	// change state
	if (GPIO_Pin == BTN1_Pin) {
		changeStateMode();
	} else if (GPIO_Pin == BTN2_Pin || GPIO_Pin == SYNC_Pin) {
		if (isWaitingForSync()) {
			state.preparedToRunMeasurementPhase = 1;
		}
	}
}
