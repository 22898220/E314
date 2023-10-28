/*
 * stateManagement.h
 *
 *  Created on: 10 Mar 2022
 *      Author: seanm
 */
#pragma once

#include "main.h"
#include "lcd.h"

// Measurement modes
#define STATE_MEAS_DV 1
#define STATE_MEAS_DI 2
#define STATE_MEAS_AV 3
#define STATE_MEAS_AI 4
#define STATE_MEAS_TC 5

// Output switch
#define STATE_OUTPUT_ON 1
#define STATE_OUTPUT_OFF 0

// Output type
#define STATE_OUTPUT_TYPE_DC 1
#define STATE_OUTPUT_TYPE_SINUSOIDAL 2
#define STATE_OUTPUT_TYPE_PULSE 3

// Measurement param
#define STATE_MEAS_PARAM_T 1
#define STATE_MEAS_PARAM_A 2
#define STATE_MEAS_PARAM_O 3
#define STATE_MEAS_PARAM_F 4
#define STATE_MEAS_PARAM_D 5
#define STATE_MEAS_PARAM_C 6

// LCD state
#define STATE_LCD_MENU 1
#define STATE_LCD_DISPLAY 2
#define STATE_LCD_CHAR 3
#define STATE_LCD_CMD 4

uint8_t state;
uint8_t outputState;
uint8_t outputType;
uint8_t measurementParameter;
uint8_t LCDState;

void setState(uint8_t newState);
void setOutType(uint8_t newOutType);
void setOutState(uint8_t newOutState);
void setMeasParam(uint8_t newMeasParam);
void setLCDState(uint8_t newLCDState);
const char* stateToString();
uint8_t outTypeToString();
uint8_t outputToString();
uint8_t measParamToString();
void initState();


/* INC_STATEMANAGEMENT_H_ */
