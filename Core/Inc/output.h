/*
 * output.h
 *
 *  Created on: Apr 22, 2022
 *      Author: Lee Johnson 24058661
 */

#include "main.h"
#include "stateManagement.h"
#include <stdlib.h>
#include "math.h"
#include "lcd.h" // Debug purposes

#define SIN_BUFFER_SIZE 2500
#define PULSE_BUFFER_SIZE 100
#define DC_BUFFER_SIZE 100

extern DAC_HandleTypeDef hdac1;
extern DMA_HandleTypeDef hdma_dac1_ch1;

uint32_t sinBuffer[SIN_BUFFER_SIZE];
uint32_t dc_val[DC_BUFFER_SIZE];
uint32_t pulseBuffer[DC_BUFFER_SIZE];

uint8_t pulseBufferChanged;
uint8_t pulseOutputOn;
uint8_t sinBufferChanged;
uint8_t sinOutputOn;
uint8_t offsetChanged;
uint8_t dcOutputOn;
uint8_t changedParam;

uint8_t acOffsetChanged;
uint8_t amplitudeChanged;
uint8_t frequencyChanged;

uint8_t pulseOffsetChanged;
uint8_t dutyChanged;

float dcOffset;
float acOffset;
float pulseOffset;
float amplitude;
float frequency;
uint16_t duty;
float outputDCOffset;
float outputACOffset;
float outputAmplitude;
float outputFrequency;

float getValue(char *stringValue);
float calibrate(float sample);
void updateOffset(float newOffset);
void updateAmplitude(float newAmplitude);
void updateFrequency(float newFrequency);
void updateDuty(uint16_t newDuty);
void dcOut();
void calcSin();
void acOut();
void pulseOut();
