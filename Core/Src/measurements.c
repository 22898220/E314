/*
 * measurements.c
 *
 *  Created on: 09 Mar 2022
 *      Author: seanm
 */

#include "measurements.h"

void sampleADC() {
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	ADCBuffer[ADCBufferOffset] = HAL_ADC_GetValue(&hadc1);
	ADCBufferOffset++;
	if (ADCBufferOffset >= ADC_BUFFER_SIZE) {
		ADCBufferOffset = 0;
	}
}

uint16_t measureAmplitude() {
	const uint8_t arrSize = 30;
	uint16_t min[arrSize];
	uint16_t max[arrSize];

	for (int i = 0; i < arrSize; i++) {
		min[i] = 4095;
		max[i] = 0;
	}

	for (int i = 0; i < ADC_BUFFER_SIZE; i++) {
		for (int j = 0; j < arrSize; j++) {
			if (min[j] > ADCBuffer[i]) {
				min[j] = ADCBuffer[i];
				break;
			}
		}
		for (int j = 0; j < arrSize; j++) {
			if (max[j] < ADCBuffer[i]) {
				max[j] = ADCBuffer[i];
				break;
			}
		}
	}
	float aveMax = 0;
	float aveMin = 0;
	for (int i = 0; i < arrSize; i++) {
		aveMax += max[i];
		aveMin += min[i];
	}
	aveMax = aveMax / (float) arrSize;
	aveMin = aveMin / (float) arrSize;
	uint16_t amplitude = (uint16_t) (aveMax - aveMin);
	if(amplitude < 105){
	return amplitude*(float)1.5;
	}
	else if((105 <= amplitude)&&(amplitude < 401)){
	return amplitude*(float)1.1;
	}
	else if ((401 <= amplitude)&&(amplitude < 575)){
		return amplitude*(float)0.95;
	}
	else if ((575 <= amplitude)&&(amplitude < 800)){
		return amplitude*(float)0.93;
	}
	else if ((800 <= amplitude)&&(amplitude < 1200)){
			return amplitude*(float)0.90;
	}
	else if ((1200 <= amplitude)&&(amplitude < 1600)){
			return amplitude*(float)0.85;
	}
	else if ((1600 <= amplitude)&&(amplitude < 2100)){
			return amplitude*(float)0.80;
	}
}

uint16_t measureDCOffset() {
	uint32_t count = 0;
	for (int i = 0; i < ADC_BUFFER_SIZE; i++) {
		count += ADCBuffer[i];
	}
	float average = count / ((float) ADC_BUFFER_SIZE);
	uint16_t offset = (uint16_t) average;
	return offset*(float)0.9;
}

uint16_t measureFrequency() {
	const uint8_t arrSize = 30;
	uint16_t numPoints = 0;
	uint16_t offset = measureDCOffset();
	uint8_t state;
	uint8_t previousState;
	uint16_t crossingPoints[arrSize];
	uint8_t noiseMargin = 10;

	if (ADCBuffer[ADCBufferOffset] - offset > 5) {
		state = 1;
		previousState = state;
	}
	if (ADCBuffer[ADCBufferOffset] - offset < -5) {
		state = 0;
		previousState = state;
	}

	for (int count = 1; count < ADC_BUFFER_SIZE; count++) {
		int i = (count + ADCBufferOffset) % ADC_BUFFER_SIZE;
		if (ADCBuffer[i] - offset > noiseMargin) {
			state = 1;
		}
		if (ADCBuffer[i] - offset < -noiseMargin) {
			state = 0;
		}
		if (state != previousState) {
			previousState = state;
			for (int j = arrSize - 1; j > 0; j--) {
				crossingPoints[j] = crossingPoints[j - 1];
			}
			crossingPoints[0] = count;
			numPoints++;
		}
	}

	float frequency = 0;
	if (numPoints > arrSize) {
		numPoints = arrSize;
	}
	for (int i = 0; i < numPoints - 1; i++) {
		frequency += (crossingPoints[i] - crossingPoints[i + 1]);
	}
	frequency = frequency / (numPoints - 1);
	frequency = 1 / (2 * frequency / 20000);
	if(frequency < 2500){
	return (uint16_t) frequency* (float)0.95;
	}
	else if(2500<frequency ){
		return (uint16_t) frequency* (float)0.9;
	}

}

