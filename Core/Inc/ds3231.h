// ds3231.h
#ifndef DS3231_H
#define DS3231_H

#include "main.h"

typedef struct {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hour;
    uint8_t dayofweek;
    uint8_t dayofmonth;
    uint8_t month;
    uint8_t year;
} TIME;

// Declare the global time variable as external
extern TIME time;

//void servo_init(TIM_HandleTypeDef* htim, uint32_t channel);
void Debug_Print(char *message);
uint8_t decToBcd(int val);
int bcdToDec(uint8_t val);
void Set_Time(uint8_t sec, uint8_t min, uint8_t hour, uint8_t dow, uint8_t dom, uint8_t month, uint8_t year);
void Set_init_time(char *date, char *time);
void Get_Time(void);

#endif // DS3231_H
