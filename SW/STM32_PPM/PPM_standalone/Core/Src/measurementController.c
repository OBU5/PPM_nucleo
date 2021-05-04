/*
 * measurementController.c
 *
 *  Created on: Apr 30, 2021
 *      Author: OBU
 */


#include "measruementController.h"
#include"main.h"


void goToIdleAfterMeasurement() {
	//only if all measurements were done
	if ((state.extAdcMeasuring == 0) && (state.intAdcMeasuring == 0) && (state.compMeasuring == 0)) {
		switchingCircuitIdle();
		set_LED1(0, 0, 0);
		state.index++;
		// -1 indicates infinity measurements
		if (canDecreaseRemainingMeasurements()) {
			state.remainingMeasurements--;
			//if this was the last measurement - set all states to 0
			if (state.remainingMeasurements == 0) {
				state.extAdcActiveState = 0;
				state.extAdcSetState = 0;
				state.intAdcActiveState = 0;
				state.intAdcSetState = 0;
				state.compActiveState = 0;
				state.compSetState = 0;
			}
		}
	}
}



void setMeasurementPeriod(uint16_t time) {
	if (time > 2000) {
		state.polarizationPeriod = time - 2100;
		state.wholeMeasurementPeriod = time;
	}
}

