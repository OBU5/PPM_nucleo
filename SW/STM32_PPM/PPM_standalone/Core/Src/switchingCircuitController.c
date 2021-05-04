/*
 * switchingController.c
 *
 *  Created on: Apr 30, 2021
 *      Author: OBU
 */

#include "switchingCircuitController.h"

void runPolarizationSequence() {

	//polarization phase will be ready after measurements
	state.preparedToRunPolarizationPhase = 0;
	remainingTimeToNextMeasurement = state.wholeMeasurementPeriod * 1000; // convert ms to us
	remainingPolarizationTime = state.polarizationPeriod * 1000; // convert ms to us

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
	state.preparedToRunMeasurementPhase = 1;

}

void runMeasurementSequence() {
	state.preparedToRunMeasurementPhase = 0;
	set_LED1(0, 1, 0);

	//run sequnece T4 - Coil discharge
	HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 0);
	HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 0);
	HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, 1);
	HAL_GPIO_WritePin(S4_GPIO_Port, S4_Pin, 0);
	HAL_GPIO_WritePin(S5_GPIO_Port, S5_Pin, 0);
	HAL_GPIO_WritePin(S6_GPIO_Port, S6_Pin, 1);
	delay_us(50);

	//run sequnece T5 - Coil discharge
	HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 0);
	HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 0);
	HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, 1);
	HAL_GPIO_WritePin(S4_GPIO_Port, S4_Pin, 0);
	HAL_GPIO_WritePin(S5_GPIO_Port, S5_Pin, 1);
	HAL_GPIO_WritePin(S6_GPIO_Port, S6_Pin, 1);
	delay_ms(10);

	//run sequnece T7 - wait before measuring
	HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 0);
	HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 0);
	HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, 1);
	HAL_GPIO_WritePin(S4_GPIO_Port, S4_Pin, 0);
	HAL_GPIO_WritePin(S5_GPIO_Port, S5_Pin, 1);
	HAL_GPIO_WritePin(S6_GPIO_Port, S6_Pin, 0);
	delay_ms(10);

	//run sequnece T8 - wait before measuring
	HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 0);
	HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 0);
	HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, 0);
	HAL_GPIO_WritePin(S4_GPIO_Port, S4_Pin, 0);
	HAL_GPIO_WritePin(S5_GPIO_Port, S5_Pin, 1);
	HAL_GPIO_WritePin(S6_GPIO_Port, S6_Pin, 0);
	delay_ms(10);

	//run sequnece T9 - measure
	HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 0);
	HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 1);
	HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, 0);
	HAL_GPIO_WritePin(S4_GPIO_Port, S4_Pin, 0);
	HAL_GPIO_WritePin(S5_GPIO_Port, S5_Pin, 1);
	HAL_GPIO_WritePin(S6_GPIO_Port, S6_Pin, 0);
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

