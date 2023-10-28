/*
 * uart_handler.h
 *
 *  Created on: 06 Mar 2022
 *      Author: seanm
 */
#pragma once

#include "main.h"
#include "stateManagement.h"
#include "measurements.h"
#include "lcd.h"
#include "output.h"
#include <string.h>

extern UART_HandleTypeDef huart2;

// Buffers
uint8_t input_message[15]; // Input message buffer
uint8_t mes_mode[2]; // Two element  buffer including the current measurement mode
uint8_t status_message[11]; // Message sent if status is requested
uint8_t measure_message[13]; // Message sent if measurement is requested
uint8_t rxData[1];
uint8_t paramValue[4];

// Buffer offsets
uint8_t input_length;

// Flags
uint8_t send_status;
uint8_t send_measure;
uint8_t input_complete;
uint8_t changeLine;

//Debug LEDS
uint8_t ledD2;
uint8_t ledD3;
uint8_t ledD4;
uint8_t ledD5;

// Measurement Values
uint16_t measAmplitude;
uint16_t measOffset;
uint16_t measFrequency;

void process_input();
void sendMeasure();
void debugLeds();
