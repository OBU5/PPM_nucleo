/*
 * config.h
 *
 *  Created on: Apr 30, 2021
 *      Author: OBU
 */

#include <stdint.h>

#ifndef SRC_CONFIG_H_
#define SRC_CONFIG_H_

#define samplesPerPeriod  44100

typedef struct States {
	uint8_t waitForSync;			//

	uint8_t extAdcReadyToSend; 		//
	uint8_t intAdcReadyToSend; 		//
	uint8_t compReadyToSend; 		//

	uint8_t extAdcActiveState; 		//
	uint8_t intAdcActiveState; 		//
	uint8_t compActiveState; 		//

	uint8_t extAdcMeasuring; 		//
	uint8_t intAdcMeasuring; 		//
	uint8_t compMeasuring; 			//

	uint8_t extAdcSetState; 		//
	uint8_t intAdcSetState; 		//
	uint8_t compSetState; 			//

	int16_t remainingMeasurements; 	//
	int16_t setMeasurements; 		//
	int16_t wholeMeasurementPeriod; //in ms
	int16_t polarizationPeriod; 	//in ms

	uint8_t newDataInBuffer;
	uint8_t measureTechniqueUpdated;
	uint8_t preparedToRunPolarizationPhase;
	uint8_t preparedToRunMeasurementPhase;
	uint32_t index;
} State;

union Buffer {
	uint8_t uint8[44100 * 2];
	uint16_t uint16[44100];
};

//State of
State state;


uint8_t buffer_uart_rx[1];

union Buffer buffer_extAdc_1;
union Buffer buffer_extAdc_2;
union Buffer buffer_intAdc_1;
union Buffer buffer_intAdc_2;

uint32_t buffer_comp[4001];
uint8_t receivedChars[100];

uint32_t timeIndex;

uint32_t remainingTimeToNextMeasurement;
uint32_t remainingPolarizationTime;


void initialization();

#endif /* SRC_CONFIG_H_ */
