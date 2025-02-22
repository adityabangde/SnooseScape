#ifndef INTERRUPT_H
#define INTERRUPT_H

#include "main.h"


#define MAX_HOUR    23
#define MAX_MINUTE  59
#define MAX_MONTH   12
#define MAX_DAY     31
#define MIN_YEAR    0
#define MAX_YEAR    99  // For 2-digit year

extern UART_HandleTypeDef huart2;

int validateTime(int hour, int minute);
int validateDate(int month, int day, int year);
int parseTimeString(const char* str, int* hour, int* minute);
int parseDateString(const char* str, int* month, int* day, int* year);
void processTimeData(uint8_t* data, uint8_t* HH, uint8_t* mm, uint8_t* DD, uint8_t* MM, uint8_t* YY, int* timer_type);

#endif // INTERRUPT_H
