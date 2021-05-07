/*
 * communication.c
 *
 *  Created on: Apr 30, 2021
 *      Author: OBU
 */
#include "stm32f7xx_hal.h"
#include "config.h"
#include "communication.h"

int parseText(UART_HandleTypeDef huart) {
	//-------------------------------------------------------------------------------------
	//check if there is character "<" and ">"
	//-------------------------------------------------------------------------------------
	state.newDataInBuffer = 0;

	uint8_t i, indexOfHead, indexOfTail, tailCount = 0, headCount = 0;
	char msg_buffers[80];
	char receivedCommand[50];

	for (i = 0; i < strlen(receivedCommand); i++) {
		receivedCommand[i] = '\0';
	}
	for (i = 0; i < strlen(msg_buffers); i++) {
		msg_buffers[i] = '\0';
	}
	for (i = 0; i < strlen(receivedChars); i++) {
		//head of the message
		if (receivedChars[i] == '<') {
			indexOfHead = i;
			headCount++;
		}
		//tail of the message
		else if (receivedChars[i] == '>') {
			indexOfTail = i;
			tailCount++;
		}
	}
	//received message is not complete
	if (headCount > tailCount) {
		return 0;
	}
	// received message is complete ->
	else if (headCount == tailCount && headCount > 0 && tailCount > 0) {
		sprintf(msg_buffers, "<INFO:Message accepted>\n");
		HAL_UART_Transmit(&huart, (uint8_t*) msg_buffers, strlen(msg_buffers), HAL_MAX_DELAY);
	}
	//received message is wrong -> delete it
	else if ((headCount < tailCount)) {
		clearReceivedCharsBuffer(); // receivedChars needs to be cleared
		return 0;
	} else {
		return 0;
	}

	//-------------------------------------------------------------------------------------
	// get string between special chars (Only if specialCharCount == 2)
	//-------------------------------------------------------------------------------------
	strncpy(receivedCommand, receivedChars + indexOfHead + 1, indexOfTail - indexOfHead - 1);
	receivedCommand[indexOfTail - indexOfHead - 1] = '\0';

	char *command = strtok(receivedCommand, ":");
	char *method = strtok(NULL, ":");
	char *count = strtok(NULL, ":");

	//-------------------------------------------------------------------------------------
	// Perform action based on received message
	//-------------------------------------------------------------------------------------

	//<IDN> - identification
	if (strcmp(command, "IDN") == 0) {
		sprintf(msg_buffers, "<INFO:This is proton precession magnetometer - version 1>\n");
		HAL_UART_Transmit(&huart, (uint8_t*) msg_buffers, strlen(msg_buffers), HAL_MAX_DELAY);
	}

	//<IDLE> - initialization state
	if (strcmp(command, "IDLE") == 0) {
		setStateToIdle();
	}
	//<DEFAULT> - initialization state
	if (strcmp(command, "DEFAULT") == 0) {
		setStateToDefault();
	}
	//<SET:parameter:value>
	if (strcmp(command, "SET") == 0) {
		//polarization time
		if (strcmp(method, "period") == 0) {
			//convert received string to integer
			uint32_t tmpVal = atoi(count);
			if(tmpVal > 5000 && tmpVal < 8640000){
				setMeasurementPeriod(tmpVal);
			}
		}
	}
	//<MEAS:method:count>
	else if (strcmp(command, "MEAS") == 0) {
		//external ADC only
		if (strcmp(method, "extADC") == 0) {
			state.extAdcSetState = 1;
			state.intAdcSetState = 0;
			state.compSetState = 0;
			state.measureTechniqueUpdated = 1;
			setMeasurementPeriod(8000);
		}
		//internal ADC only
		else if (strcmp(method, "intADC") == 0) {
			state.extAdcSetState = 0;
			state.intAdcSetState = 1;
			state.compSetState = 0;
			state.measureTechniqueUpdated = 1;
			setMeasurementPeriod(8000);
		}
		//comparator only
		else if (strcmp(method, "comp") == 0) {
			state.extAdcSetState = 0;
			state.intAdcSetState = 0;
			state.compSetState = 1;
			state.measureTechniqueUpdated = 1;
			setMeasurementPeriod(5000);
		}
		//external ADC + internal ADC
		else if ((strcmp(method, "extADC+intADC")) == 0 || (strcmp(method, "intADC+extADC")) == 0) {
			state.extAdcSetState = 1;
			state.intAdcSetState = 1;
			state.compSetState = 0;
			state.measureTechniqueUpdated = 1;
			setMeasurementPeriod(13000);
		}
		//external ADC + comparator
		else if ((strcmp(method, "extADC+comp") == 0) || (strcmp(method, "comp+extADC") == 0)) {
			state.extAdcSetState = 1;
			state.intAdcSetState = 0;
			state.compSetState = 1;
			state.measureTechniqueUpdated = 1;
			setMeasurementPeriod(8000);
		}
		//internal ADC + comparator
		else if ((strcmp(method, "intADC+comp") == 0) || (strcmp(method, "comp+intADC")) == 0) {
			state.extAdcSetState = 0;
			state.intAdcSetState = 1;
			state.compSetState = 1;
			state.measureTechniqueUpdated = 1;
			setMeasurementPeriod(8000);
		}
		//external ADC + internal ADC + comparator
		else if ((strcmp(method, "extADC+intADC+comp") == 0) || (strcmp(method, "intADC+extADC+comp") == 0)) {
			state.compSetState = 1;
			state.extAdcSetState = 1;
			state.intAdcSetState = 1;
			state.measureTechniqueUpdated = 1;
			setMeasurementPeriod(13000);
		} else /* default: */
		{

		}
		if (strcmp(count, "INF") == 0) {
			state.setMeasurements = -1;
		} else if (strcmp(count, "") == 0) {
			state.setMeasurements = 1;
		} else if (strcmp(count, "onSync") == 0) {
			setStateToMeasureOnSync();
		}

		else {
			state.setMeasurements = atoi(count);
		}

	}
	/* more else if clauses */
	else /* default: */
	{
	}
	clearReceivedCharsBuffer();
}


void sendMeasuredData(UART_HandleTypeDef huart) {
	char msg_freq[16];
	char msg_buffers[50];
	uint16_t adc = 0;
	int i = 0;
	if ((state.extAdcReadyToSend == 1)) {
		sprintf(msg_buffers, "<MEAS:%u:extADC:\n", state.index);
		HAL_UART_Transmit(&huart, (uint8_t*) msg_buffers, strlen(msg_buffers), HAL_MAX_DELAY);

		// first buffer

		for (i = 0; i < samplesPerPeriod; i++) {
			adc = (buffer_extAdc_1.uint16[i]);
			sprintf(msg_buffers, "%hu\n", adc);
			HAL_UART_Transmit(&huart, (uint8_t*) msg_buffers, strlen(msg_buffers), HAL_MAX_DELAY);
		}
		//second buffer
		for (i = 0; i < samplesPerPeriod; i++) {
			adc = (buffer_extAdc_2.uint16[i]);
			sprintf(msg_buffers, "%hu\n", adc);
			HAL_UART_Transmit(&huart, (uint8_t*) msg_buffers, strlen(msg_buffers), HAL_MAX_DELAY);
		}
		sprintf(msg_buffers, ">\n");
		HAL_UART_Transmit(&huart, (uint8_t*) msg_buffers, strlen(msg_buffers), HAL_MAX_DELAY);
		state.extAdcReadyToSend = 0;
	}

	if ((state.intAdcReadyToSend == 1)) {
		sprintf(msg_buffers, "<MEAS:%u:intADC:\n", state.index);
		HAL_UART_Transmit(&huart, (uint8_t*) msg_buffers, strlen(msg_buffers), HAL_MAX_DELAY);
		// first buffer
		for (i = 0; i < samplesPerPeriod; i++) {
			adc = (buffer_intAdc_1.uint16[i]);
			sprintf(msg_buffers, "%hu\n", adc);
			HAL_UART_Transmit(&huart, (uint8_t*) msg_buffers, strlen(msg_buffers), HAL_MAX_DELAY);
		}
		//second buffer
		for (i = 0; i < samplesPerPeriod; i++) {
			adc = (buffer_intAdc_2.uint16[i]);
			sprintf(msg_buffers, "%hu\n", adc);
			HAL_UART_Transmit(&huart, (uint8_t*) msg_buffers, strlen(msg_buffers), HAL_MAX_DELAY);
		}
		sprintf(msg_buffers, ">\n");
		HAL_UART_Transmit(&huart, (uint8_t*) msg_buffers, strlen(msg_buffers), HAL_MAX_DELAY);
		state.intAdcReadyToSend = 0;
	}

	if ((state.compReadyToSend == 1)) {
		//send frequency
		sprintf(msg_buffers, "<MEAS:%u:comp:\n", state.index);
		HAL_UART_Transmit(&huart, (uint8_t*) msg_buffers, strlen(msg_buffers), HAL_MAX_DELAY);
		for (i = 0; i < 4000; i++) {
			uint32_t freq = buffer_comp[i + 1] - buffer_comp[i];
			sprintf(msg_freq, "%d\n", freq);
			HAL_UART_Transmit(&huart, (uint8_t*) msg_freq, strlen(msg_freq), HAL_MAX_DELAY);
		}
		sprintf(msg_buffers, ">\n");
		HAL_UART_Transmit(&huart, (uint8_t*) msg_buffers, strlen(msg_buffers), HAL_MAX_DELAY);
		state.compReadyToSend = 0;
	}
}

int newDataInBuffer() {
	return (state.newDataInBuffer);
}


int dataReadyToSend() {
	return (state.extAdcReadyToSend || state.intAdcReadyToSend || state.compReadyToSend);
}

//data should be sent after measurement only in "mode 1" or when it is the last measurement
int dataReadyToSendAfterMeasurement() {
	return (dataReadyToSend() && (lastMeasurement() || state.waitForSync == 1));
}

//data should be sent before measurement only in "mode 0", when it's just right after polarization
int dataReadyToSendBeforeMeasurement() {
	return (dataReadyToSend() && state.waitForSync == 0);
}

void clearReceivedCharsBuffer() {
	int i = 0;
	//keep everything in range
	if (receivedCharIndex > 50) {
		receivedCharIndex = 50;
	}
	for (i = 0; i < receivedCharIndex; i++) {
		receivedChars[i] = '\0';
	}
	receivedCharIndex = 0;
}






