#include "stm32f4xx.h"                
#ifndef __FUNC_H
#define __FUNC_H

void reverse(char* str, int len);
void ftoa(float n, char* res, int afterpoint);
int intToStr(int x, char str[], int d);
uint8_t hex_to_deci(char hex[] );
void Delay(int);
#endif
