/*
 * stateController.c
 *
 *  Created on: Apr 30, 2021
 *      Author: OBU
 */

#include "stateController.h"

void changeStateMode() {
	if (state.waitForSync == 0) {
		state.extAdcSetState = 1;
		state.intAdcSetState = 0;
		state.compSetState = 0;

		state.extAdcActiveState = 1;
		state.intAdcActiveState = 0;
		state.compActiveState = 0;

		setStateToMeasureOnSync();
	} else if (state.waitForSync == 1) {
		state.waitForSync = 0;

	}
}

int stateCanBeUpdated() {
	return (state.measureTechniqueUpdated && !state.extAdcMeasuring && !state.intAdcMeasuring && !state.compMeasuring && !state.extAdcReadyToSend && !state.intAdcReadyToSend && !state.compReadyToSend);
}

void updateState() {
	state.remainingMeasurements = state.setMeasurements;
	state.extAdcActiveState = state.extAdcSetState;
	state.intAdcActiveState = state.intAdcSetState;
	state.compActiveState = state.compSetState;
	state.preparedToRunPolarizationPhase = 1;
	state.measureTechniqueUpdated = 0;
}


void setStateToDefault() {
	state.waitForSync = 0;
	state.extAdcReadyToSend = 0;
	state.intAdcReadyToSend = 0;
	state.compReadyToSend = 0;

	state.extAdcActiveState = 1;
	state.intAdcActiveState = 0;
	state.compActiveState = 1;

	state.extAdcMeasuring = 0;
	state.intAdcMeasuring = 0;
	state.compMeasuring = 0;

	state.extAdcSetState = 1;
	state.intAdcSetState = 0;
	state.compSetState = 0;

	state.remainingMeasurements = -1;
	state.setMeasurements = -1;
	setMeasurementPeriod(8000);		//8 sec

	state.newDataInBuffer = 0;
	state.measureTechniqueUpdated = 0;
	state.preparedToRunPolarizationPhase = 1;
	state.preparedToRunMeasurementPhase = 0;
	state.index = 0;

}

void setStateToMeasureOnSync() {
	state.waitForSync = 1;

	state.extAdcReadyToSend = 0;
	state.intAdcReadyToSend = 0;
	state.compReadyToSend = 0;

	state.extAdcActiveState = state.extAdcSetState;
	state.intAdcActiveState = state.intAdcSetState;
	state.compActiveState = state.compSetState;

	state.extAdcMeasuring = 0;
	state.intAdcMeasuring = 0;
	state.compMeasuring = 0;

	state.remainingMeasurements = -1;
	state.setMeasurements = -1;
	state.wholeMeasurementPeriod = 0; 	//in ms -> 5 sec
	state.polarizationPeriod = 0; 		//in ms -> 3 sec

	remainingTimeToNextMeasurement = 0;
	remainingPolarizationTime = 0;
	setMeasurementPeriod(2000);

	state.newDataInBuffer = 0;
	state.measureTechniqueUpdated = 0;
	state.preparedToRunPolarizationPhase = 0;
	state.preparedToRunMeasurementPhase = 0;
	state.index = 0;

	set_LED1(0, 0, 0);
	switchingCircuitIdle();

}

void setStateToIdle() {
	state.extAdcReadyToSend = 0;
	state.intAdcReadyToSend = 0;
	state.compReadyToSend = 0;

	state.extAdcActiveState = 0;
	state.intAdcActiveState = 0;
	state.compActiveState = 0;

	state.extAdcMeasuring = 0;
	state.intAdcMeasuring = 0;
	state.compMeasuring = 0;

	state.extAdcSetState = 0;
	state.intAdcSetState = 0;
	state.compSetState = 0;

	state.remainingMeasurements = 0;
	state.setMeasurements = 0;
	state.wholeMeasurementPeriod = 0; 	//in ms -> 5 sec
	state.polarizationPeriod = 0; 		//in ms -> 3 sec

	state.measureTechniqueUpdated = 0;
	state.preparedToRunPolarizationPhase = 0;
	state.preparedToRunMeasurementPhase = 0;
	state.index = 0;
}


int lastMeasurement() {
	return (state.remainingMeasurements == 0);
}


// polarization can run even if data are sending
int ploarizationCanRun() {
	return (state.waitForSync == 0 && remainingTimeToNextMeasurement == 0 && state.preparedToRunPolarizationPhase && !state.extAdcMeasuring && !state.intAdcMeasuring && !state.compMeasuring);
}

// measurement sequence can run if polarization can run and all the data
int measurementCanRun() {
	return (remainingPolarizationTime == 0 && state.preparedToRunMeasurementPhase && !state.extAdcMeasuring && !state.intAdcMeasuring && !state.compMeasuring && !state.extAdcReadyToSend
			&& !state.intAdcReadyToSend && !state.compReadyToSend);
}

int stateIsIdle() {
	return (!state.extAdcActiveState && !state.intAdcActiveState && !state.compActiveState && !state.extAdcReadyToSend && !state.intAdcReadyToSend && !state.compReadyToSend);
}

int isWaitingForSync() {
	return (state.waitForSync == 1 && !isMeasuring() && !isSending());
}

int  isMeasuring(){
	return (state.extAdcMeasuring || state.intAdcMeasuring || state.compMeasuring);
}
int isSending() {
	return (state.extAdcReadyToSend || state.intAdcReadyToSend || state.compReadyToSend);
}

int canDecreaseRemainingMeasurements() {
	return ((state.remainingMeasurements > 0) && (state.intAdcActiveState || state.extAdcActiveState || state.compActiveState));
}

int isModeMeasureOnSync() {
	return (state.waitForSync == 1);
}


void prepareForPolarizationPhaseIfPossible() {
	if ((state.waitForSync == 0) && (state.remainingMeasurements != 0) && (state.extAdcMeasuring == 0) && (state.intAdcMeasuring == 0) && (state.compMeasuring == 0)) {
		state.preparedToRunPolarizationPhase = 1;
	}
}
