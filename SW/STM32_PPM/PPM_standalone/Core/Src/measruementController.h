/*
 * measruementController.h
 *
 *  Created on: Apr 30, 2021
 *      Author: OBU
 */

#include "config.h"
#include "main.h"

#ifndef SRC_MEASRUEMENTCONTROLLER_H_
#define SRC_MEASRUEMENTCONTROLLER_H_


void runMeasurementMethod();

void measureWithExtADC();

void measureWithIntADC() ;

void measureWithComp();
void measurementWithExtAdcDone();
void measurementWithIntAdcDone();
void measurementWithCompDone(TIM_HandleTypeDef *htim);
void goToIdleAfterMeasurement();

//comparator finished measuring
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

//intADC - buffer filled
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);

//char received with via USB
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

//extADC - buffer filled
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);

void setMeasurementPeriod(uint16_t time);

#endif /* SRC_MEASRUEMENTCONTROLLER_H_ */
