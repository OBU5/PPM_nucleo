/*
 * delay.c
 *
 *  Created on: Apr 30, 2021
 *      Author: OBU
 */

#include "delay.h"
#include"main.h"

void delay_us(uint32_t delay_us) {
	timeIndex = delay_us;
	while (timeIndex > 0);
}

void delay_ms(uint32_t delay_us) {
	timeIndex = delay_us * 1000;
	while (timeIndex > 0);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM5) {
		if (timeIndex > 0) {
			timeIndex -= 50;
		}
		if (remainingTimeToNextMeasurement > 0) {
			remainingTimeToNextMeasurement -= 50;
		} else {
			prepareForPolarizationPhaseIfPossible();
		}
		if (remainingPolarizationTime > 0) {
			remainingPolarizationTime -= 50;
		}
	}
}
