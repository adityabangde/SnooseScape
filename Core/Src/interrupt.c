#include "interrupt.h"

// Function to validate time components
int validateTime(int hour, int minute) {
    if (hour < 0 || hour > MAX_HOUR) return 0;
    if (minute < 0 || minute > MAX_MINUTE) return 0;
    return 1;
}

// Function to validate date components
int validateDate(int month, int day, int year) {
    if (month < 1 || month > MAX_MONTH) return 0;
    if (day < 1 || day > MAX_DAY) return 0;
    if (year < MIN_YEAR || year > MAX_YEAR) return 0;

    // Check days in month (including February in leap year)
    int daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (year % 4 == 0) daysInMonth[1] = 29;  // Leap year

    if (day > daysInMonth[month-1]) return 0;

    return 1;
}

int parseTimeString(const char* str, int* hour, int* minute) {
    // Check if we have at least 5 chars for HH:MM
    if (strlen(str) < 5) return 0;

    // Parse hours
    *hour = (str[0] - '0') * 10 + (str[1] - '0');

    // Check for colon
    if (str[2] != ':') return 0;

    // Parse minutes
    *minute = (str[3] - '0') * 10 + (str[4] - '0');

    return 1;
}

int parseDateString(const char* str, int* month, int* day, int* year) {
    // Check if we have at least 8 chars for MM:DD:YY
    if (strlen(str) < 8) return 0;

    // Parse month
    *month = (str[0] - '0') * 10 + (str[1] - '0');

    // Check for colon
    if (str[2] != ':') return 0;

    // Parse day
    *day = (str[3] - '0') * 10 + (str[4] - '0');

    // Check for colon
    if (str[5] != ':') return 0;

    // Parse year
    *year = (str[6] - '0') * 10 + (str[7] - '0');

    return 1;
}

void processTimeData(uint8_t* data, uint8_t* HH, uint8_t* mm, uint8_t* DD, uint8_t* MM, uint8_t* YY, int* timer_type)
{
	char debug_msg[50];
	    int hour = 0, minute = 0, month = 0, day = 0, year = 0;

	    // Check first character for command type
	    if(data[0] == 't') {
	        // Get hours (skip 't' and space)
	        hour = (data[2] - '0') * 10 + (data[3] - '0');

	        // Check colon
	        if(data[4] != ':') {
	            HAL_UART_Transmit(&huart2, (uint8_t*)"Wrong format\r\n", 13, HAL_MAX_DELAY);
	            return;
	        }

	        // Get minutes
	        minute = (data[5] - '0') * 10 + (data[6] - '0');

	        // Check space
	        if(data[7] != ' ') {
	            HAL_UART_Transmit(&huart2, (uint8_t*)"Wrong format\r\n", 13, HAL_MAX_DELAY);
	            return;
	        }

	        // Get month
	        month = (data[8] - '0') * 10 + (data[9] - '0');

	        // Check colon
	        if(data[10] != ':') {
	            HAL_UART_Transmit(&huart2, (uint8_t*)"Wrong format\r\n", 13, HAL_MAX_DELAY);
	            return;
	        }

	        // Get day
	        day = (data[11] - '0') * 10 + (data[12] - '0');

	        // Check colon
	        if(data[13] != ':') {
	            HAL_UART_Transmit(&huart2, (uint8_t*)"Wrong format\r\n", 13, HAL_MAX_DELAY);
	            return;
	        }

	        // Get year
	        year = (data[14] - '0') * 10 + (data[15] - '0');

	        // Validate time
	        if(hour > 23 || minute > 59) {
	            HAL_UART_Transmit(&huart2, (uint8_t*)"Invalid time\r\n", 13, HAL_MAX_DELAY);
	            return;
	        }

	        // Validate date
	        if(month < 1 || month > 12 || day < 1 || day > 31) {
	            HAL_UART_Transmit(&huart2, (uint8_t*)"Invalid date\r\n", 13, HAL_MAX_DELAY);
	            return;
	        }

        // Set RTC

		sprintf(debug_msg, "Time set: %02d:%02d Date: %02d/%02d/%02d\r\n",
		                hour, minute, month, day, year);
		HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, strlen(debug_msg), HAL_MAX_DELAY);

        *HH = hour;
		*mm =  minute;
		*DD = day;
		*MM = month;
		*YY = year;
		*timer_type = 0;
    }
	    else if(data[0] == '#') {
	        // Get hours (skip '#')
	        hour = (data[1] - '0') * 10 + (data[2] - '0');

	        // Check colon
	        if(data[3] != ':') {
	            HAL_UART_Transmit(&huart2, (uint8_t*)"Wrong alarm format\r\n", 18, HAL_MAX_DELAY);
	            return;
	        }

	        // Get minutes
	        minute = (data[4] - '0') * 10 + (data[5] - '0');

	        // Validate time
	        if(hour > 23 || minute > 59) {
	            HAL_UART_Transmit(&huart2, (uint8_t*)"Invalid alarm time\r\n", 18, HAL_MAX_DELAY);
	            return;
	        }
	        sprintf(debug_msg, "Alarm set for: %02d:%02d\r\n", hour, minute);
	        HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, strlen(debug_msg), HAL_MAX_DELAY);

	        *HH = hour;
			*mm =  minute;
			*DD = 0;
			*MM = 0;
			*YY = 0;
			*timer_type = 1;

	    }
	    else {
	        HAL_UART_Transmit(&huart2, (uint8_t*)"Invalid command\r\n", 16, HAL_MAX_DELAY);
	    }
}
