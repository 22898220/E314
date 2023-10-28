/*
 * lcd.h
 *
 *  Created on: Apr 24, 2022
 *      Author: Lee Johnson 24058661
 */

#include "main.h"
#include "stateManagement.h"

// for debugging:
extern UART_HandleTypeDef huart2;

void LCD_DATA(unsigned char Data);
void LCD_CMD(unsigned char CMD);
void LCD_init();
void LCD_Write_Char(char Data);
void LCD_Write_String(char *str);
void LCD_Clear();
void LCD_Set_Cursor(unsigned char r, unsigned char c);
void menuView();
void displayView();
void menuInit();
void processMenu();

int node;
int level;
int previousNode;
