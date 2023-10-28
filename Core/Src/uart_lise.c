/*
 * uart_handler.c
 *
 *  Created on: 06 Mar 2022
 *      Author: Lise Prinsloo 23726059
 */

#include "uart_lise.h"

// Initializations
uint8_t status_message[] = "@,**,*,*,!\n" ;
uint8_t measure_message[] = "@,m,*,****,!\n";
uint8_t mes_mode[2] = "DV";

uint16_t measAmplitude = 0;
uint16_t measOffset = 0;
uint16_t measFrequency = 0;

uint8_t changeLine = 0;
uint8_t ledD2 = 0;
uint8_t ledD3 = 0;
uint8_t ledD4 = 0;
uint8_t ledD5 = 0;

void debugLeds(){
	if(ledD2 == 1){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
	}else HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
	if(ledD3 == 1){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
	}else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
	if(ledD4 == 1){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
	}else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
	if(ledD5 == 1){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
	}else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);

}

void process_input(){
	  //process input message and set appropriate modes
	if(input_message[2] == '$'){
		// "Set Measure Mode"
		ledD3 = 1;
		if((input_message[4]=='D')&&(input_message[5]=='V')){
			// Set measurement to DC Voltage
			mes_mode[0]= 'D';
			mes_mode[1] = 'V';
			setState(STATE_MEAS_DV);
			ledD4 = 0;
		}else if((input_message[4]=='A')&&(input_message[5]=='V')){
			// Set measurement to AC Voltage
			mes_mode[0]= 'A';
			mes_mode[1] = 'V';
			setState(STATE_MEAS_AV);
			ledD4 = 0;
		}else if((input_message[4]=='D')&&(input_message[5]=='I')){
			// Set measurement to DC Current
			mes_mode[0]= 'D';
			mes_mode[1] = 'I';
			setState(STATE_MEAS_DI);
			ledD4 = 1;
		}else if((input_message[4]=='A')&&(input_message[5]=='I')){
			// Set measurement to AC current
			mes_mode[0]= 'A';
			mes_mode[1] = 'I';
			setState(STATE_MEAS_AI);
			ledD4 = 1;
		}else if((input_message[4]=='T')&&(input_message[5]=='C')){
			// Set measurement to temperature
			mes_mode[0]= 'T';
			mes_mode[1] = 'C';
			setState(STATE_MEAS_TC);
			ledD4 = 0;
		}
	}

	if(input_message[2] == '*'){
		// Request
		if(input_message[4]=='s'){
			// Request Status
			if(input_message[6]=='0'){
				// Set output state to 0
				setOutState(STATE_OUTPUT_OFF);
				ledD5 = 0;
			} else{
				// Set output state to 1
				setOutState(STATE_OUTPUT_ON);
				ledD5 = 1;
			}
			send_status = 1 ;
		} else if(input_message[4]=='m'){
			// Request measurement mode
			send_measure = 1;
			if(input_message[6]=='t'){
				// Requesting "type"
				setMeasParam(STATE_MEAS_PARAM_T);
			}else if(input_message[6]=='a'){
				// Requesting "amplitude"
				setMeasParam(STATE_MEAS_PARAM_A);
			}else if(input_message[6]=='o'){
				// Requesting "offset"
				setMeasParam(STATE_MEAS_PARAM_O);
			}else if(input_message[6]=='f'){
				// Requesting "frequency"
				setMeasParam(STATE_MEAS_PARAM_F);
			}else if(input_message[6]=='d'){
				// Requesting "duty cycle"
				setMeasParam(STATE_MEAS_PARAM_D);
			}else if(input_message[6]=='c'){
				// Requesting "temperature"
				setMeasParam(STATE_MEAS_PARAM_C);
			}
		}

	}

	if (input_message[2] == '#')
	{
		// Display on LCD

		if(input_message[4] == '1')
		{
			// Display characters
			if(LCDState != STATE_LCD_CHAR)
			{
				if(changeLine){
					LCD_Set_Cursor(2,1);
					changeLine = 0;
				}else{
					LCD_Clear(); // Clear LCD before writing characters
				}
				setLCDState(STATE_LCD_CHAR);
			}
			LCD_Write_Char(input_message[6]);
		} else{
			// Execute commands
			setLCDState(STATE_LCD_CMD);
			// Do not clear screen before commands are executed

			uint8_t byte = input_message[6];
			char Low4, High4;
			Low4 = byte & 0x0F; // Extract only the lower 4 bits of the sent byte
			High4 = byte & 0xF0; // Extract only the upper 4 bits of the sent byte

			LCD_CMD((High4>>4)); // Send the first 4 bits to the LCD
			LCD_CMD(Low4); // Send the second 4 bits to the LCD

			if(Low4 == 0x0 && (High4 >> 4) == 0xC){
				changeLine = 1;
			}

			HAL_Delay(3); // Maximum delay needed for the LCD commands (Clear Screen -> 2.16ms at fosc = 190KHz)
		}
	}

	if(input_message[2] == '^')
	{
		// Setting output parameter
		// @,^,param,value,!\n
		// param = t, a, o, f
		// value = (d, s), (0-9999 mV), (0-9999 mV), (0-9999 Hz)
		ledD3 = 1;
		if(input_message[4] == 't'){
			// Set output type
			if(input_message[6] == 'd'){
				// DC
				setOutType(STATE_OUTPUT_TYPE_DC);
			} else if(input_message[6] == 's'){
				// AC
				setOutType(STATE_OUTPUT_TYPE_SINUSOIDAL);
			} else if (input_message[6] == 'p'){
				// Pulse
				setOutType(STATE_OUTPUT_TYPE_PULSE);
			}
		} else if(input_message[4] == 'a'){
			// Set output amplitude
			strncpy((char*) paramValue, (char*) input_message + 6, 4);
			float newAmp = getValue(paramValue);
			updateAmplitude(newAmp);
		} else if(input_message[4] == 'o'){
			// Set output offset
			strncpy((char*) paramValue, (char*) input_message + 6, 4);
			float newOff = getValue(paramValue);
			updateOffset(newOff);
		} else if(input_message[4] == 'f'){
			// Set output frequency
			strncpy((char*) paramValue, (char*) input_message + 6, 4);
			float newFreq = getValue(paramValue);
			updateFrequency(newFreq);
		}
		else if(input_message[4] == 'd'){
			// Set output pulse
			strncpy((char*) paramValue, (char*) input_message + 6, 4);
			float newDuty = getValue(paramValue);
			updateDuty((uint16_t) newDuty);
		}
	}


}

void sendMeasure(){
	// Send Measurement message
	if(state == STATE_MEAS_DV){
		// If DC: Only need to measure amplitude
		measAmplitude = measureAmplitude();
		measure_message[4]= 'a';
		measure_message[6]= (measAmplitude/1000)%10 + 48 ;
		measure_message[7]= (measAmplitude/100)%10 + 48 ;
		measure_message[8]= (measAmplitude/10)%10 + 48 ;
		measure_message[9]= (measAmplitude/1)%10 + 48 ;
	} else if(state == STATE_MEAS_AV){
		// If AC: Need to measure Amplitude, DC Offset and Frequency
//		measure_message[4]= measParamToString();
		switch (measurementParameter) {
			case STATE_MEAS_PARAM_A:
				// Amplitude measurement
				measure_message[4]= 'a';
				measAmplitude = measureAmplitude();
				measure_message[6]= (measAmplitude/1000)%10 + 48 ;
				measure_message[7]= (measAmplitude/100)%10 + 48 ;
				measure_message[8]= (measAmplitude/10)%10 + 48 ;
				measure_message[9]= (measAmplitude/1)%10 + 48 ;
				break;
			case STATE_MEAS_PARAM_O:
				// Offset measurement
				measure_message[4]= 'o';
				measOffset = measureDCOffset();
				measure_message[6]= (measOffset/1000)%10 + 48 ;
				measure_message[7]= (measOffset/100)%10 + 48 ;
				measure_message[8]= (measOffset/10)%10 + 48 ;
				measure_message[9]= (measOffset/1)%10 + 48 ;
				break;
			case STATE_MEAS_PARAM_F:
				// Frequency measurement
				measure_message[4]= 'f';
				measFrequency = measureFrequency();
				measure_message[6]= (measFrequency/1000)%10 + 48 ;
				measure_message[7]= (measFrequency/100)%10 + 48 ;
				measure_message[8]= (measFrequency/10)%10 + 48 ;
				measure_message[9]= (measFrequency/1)%10 + 48 ;
				break;
			default:
				break;
		}
	}

	HAL_UART_Transmit(&huart2, measure_message ,13,50);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	if (rxData[0] != 10){
		// If the received byte is not equal to #0x0A (Line feed)
		input_message[input_length] = rxData[0];
		input_length ++ ;
	} else{
		input_length = 0;
		input_complete = 1;
	}

	HAL_UART_Receive_IT(&huart2,rxData,1);
}
