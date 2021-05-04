/*
 * communication.h
 *
 *  Created on: Apr 30, 2021
 *      Author: OBU
 */


#include "config.h"
#ifndef SRC_COMMUNICATION_H_
#define SRC_COMMUNICATION_H_


int parseText(UART_HandleTypeDef huart);

void sendMeasuredData(UART_HandleTypeDef huart);

int newDataInBuffer();


int dataReadyToSend();

//data should be sent after measurement only in "mode 1" or when it is the last measurement
int dataReadyToSendAfterMeasurement();
//data should be sent before measurement only in "mode 0", when it's just right after polarization
int dataReadyToSendBeforeMeasurement() ;

void clearReceivedCharsBuffer();

#endif /* SRC_COMMUNICATION_H_ */
