/*
 * measurements.h
 *
 *  Created on: 09 Mar 2022
 *      Author: seanm
 */
#pragma once

#include "main.h"
#include "stdio.h"

#define ADC_BUFFER_SIZE 2000
#define ADC_TO_MV (1.16*3.3/4.095)

extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;

uint16_t ADCBuffer[ADC_BUFFER_SIZE];
uint16_t ADCBufferOffset;

void sampleADC();
uint16_t measureDCOffset();
uint16_t measureAmplitude();
uint16_t measureFrequency();

/* SRC_MEASUREMENTS_H_ */
