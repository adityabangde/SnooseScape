#include "ds3231.h"

//void servo_init(TIM_HandleTypeDef* htim, uint32_t channel){
//	HAL_TIM_PWM_Start(htim, channel);
//}
extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c3;

#define DS3231_ADDRESS 0xD0 //I2C address for RTC module
#define DEBUG_BUFFER_SIZE 100

char debug_msg[DEBUG_BUFFER_SIZE];


TIME time;

void Debug_Print(char *message)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 1000);
}

// Convert decimal to binary coded numbers
uint8_t decToBcd(int val)
{
    return (uint8_t)((val/10*16) + (val%10));
}

// Convert binary coded decimal to normal decimal numbers
int bcdToDec(uint8_t val)
{
    return (int)((val/16*10) + (val%16));
}

void Set_Time(uint8_t sec, uint8_t min, uint8_t hour, uint8_t dow, uint8_t dom, uint8_t month, uint8_t year)
{
    uint8_t set_time[7];
    set_time[0] = decToBcd(sec);
    set_time[1] = decToBcd(min);
    set_time[2] = decToBcd(hour);
    set_time[3] = decToBcd(dow);
    set_time[4] = decToBcd(dom);
    set_time[5] = decToBcd(month);
    set_time[6] = decToBcd(year);

    if(HAL_I2C_Mem_Write(&hi2c3, DS3231_ADDRESS, 0x00, 1, set_time, 7, 1000) == HAL_OK)
    {
        Debug_Print("[DEBUG] Time set successfully\r\n");
    }
    else
    {
    	Debug_Print("[DEBUG] Error setting time\r\n");
    }
}

void Set_init_time(char *date, char *time) {
//    char month_str[4];    // Buffer for month string
//	int tempYear, tempDay, tempHour, tempMinute, tempSecond;

    char month_str[4] = {date[0], date[1], date[2], 0};

	uint8_t day = ((date[4] == ' ' ? '0' : date[4]) - '0') * 10 + (date[5] - '0');
	// Get year
	uint8_t year = (date[9] - '0') * 10 + (date[10] - '0');
	// Get time
	uint8_t hour = (time[0] - '0') * 10 + (time[1] - '0');
	uint8_t min = (time[3] - '0') * 10 + (time[4] - '0');
	uint8_t sec = (time[6] - '0') * 10 + (time[7] - '0');

	uint8_t month_num = 1;
	    const char *months[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
	                           "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
	    for(int i = 0; i < 12; i++) {
	        if(strncmp(month_str, months[i], 3) == 0) {
	            month_num = i + 1;
	            break;
	        }
	    }

    // Set the time using parsed values
    Set_Time(sec, min, hour, 1, day, month_num, year);
}

void Get_Time(void)
{
    uint8_t get_time[7];
    if(HAL_I2C_Mem_Read(&hi2c3, DS3231_ADDRESS, 0x00, 1, get_time, 7, 1000) == HAL_OK)
    {
        time.seconds = bcdToDec(get_time[0]);
        time.minutes = bcdToDec(get_time[1]);
        time.hour = bcdToDec(get_time[2]);
        time.dayofweek = bcdToDec(get_time[3]);
        time.dayofmonth = bcdToDec(get_time[4]);
        time.month = bcdToDec(get_time[5]);
        time.year = bcdToDec(get_time[6]);
    }
    else
    {
        Debug_Print("[DEBUG] Error reading time\r\n");
    }
}
