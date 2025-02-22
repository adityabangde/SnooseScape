#include "main.h"

#define FILTER_SIZE 10  // Increased buffer size for better averaging

char uart_buf[200];
uint16_t filteredDistances[5];
//#define FILTER_SIZE 5  // Number of samples to average

// Add these arrays at the start of VL53L0X_Test function
uint16_t buffer1[FILTER_SIZE] = {0};
uint16_t buffer2[FILTER_SIZE] = {0};
uint16_t buffer3[FILTER_SIZE] = {0};
uint16_t buffer4[FILTER_SIZE] = {0};
uint16_t buffer5[FILTER_SIZE] = {0};

uint8_t buffer_index = 0;
#define MAX_INVALID_READINGS 3  // Maximum consecutive invalid readings

#define SENSOR1_XSHUT_PIN GPIO_PIN_5 //1st sensor from left
#define SENSOR2_XSHUT_PIN GPIO_PIN_12 //2nd sensor from left
#define SENSOR3_XSHUT_PIN GPIO_PIN_9 //3rd sensor from left
#define SENSOR4_XSHUT_PIN GPIO_PIN_6 //4th sensor from left
#define SENSOR5_XSHUT_PIN GPIO_PIN_8 //5th sensor from left

#define SENSOR_GPIO_PORT_A GPIOA
#define SENSOR_GPIO_PORT_C GPIOC

#define SENSOR1_ADDRESS 0x30
#define SENSOR2_ADDRESS 0x31
#define SENSOR3_ADDRESS 0x32
#define SENSOR4_ADDRESS 0x33
#define SENSOR5_ADDRESS 0x34

#define SYSRANGE_PART_TO_PART_RANGE_OFFSET   0x024
#define ALGO_PART_TO_PART_RANGE_OFFSET_MM    0x028

#define VL53L0X_DEFAULT_ADDRESS                     0x29
#define SYSRANGE_START                              0x00
#define SYSTEM_THRESH_HIGH                          0x0C
#define SYSTEM_THRESH_LOW                           0x0E
#define SYSTEM_SEQUENCE_CONFIG                      0x01
#define SYSTEM_RANGE_CONFIG                         0x09
#define SYSTEM_INTERMEASUREMENT_PERIOD              0x04
#define SYSTEM_INTERRUPT_CONFIG_GPIO                0x0A
#define GPIO_HV_MUX_ACTIVE_HIGH                     0x84
#define SYSTEM_INTERRUPT_CLEAR                      0x0B
#define RESULT_INTERRUPT_STATUS                     0x13
#define RESULT_RANGE_STATUS                         0x14
#define RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       0xBC
#define RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        0xC0
#define RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       0xD0
#define RESULT_CORE_RANGING_TOTAL_EVENTS_REF        0xD4
#define RESULT_PEAK_SIGNAL_RATE_REF                 0xB6
#define GLOBAL_CONFIG_VCSEL_WIDTH                   0x32
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_0            0xB0
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_1            0xB1
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_2            0xB2
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_3            0xB3
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_4            0xB4
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_5            0xB5
#define GLOBAL_CONFIG_REF_EN_START_SELECT           0xB6
#define DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         0x4E
#define DYNAMIC_SPAD_REF_EN_START_OFFSET            0x4F
#define POWER_MANAGEMENT_GO1_POWER_FORCE            0x80
#define I2C_SLAVE_DEVICE_ADDRESS                    0x8A


#define DEFAULT_SIGNAL_RATE_MCPS    0x0199    /* 0.25 MCPS */
#define DEFAULT_SIGMA_THRESH_MM     0x0060    /* 60mm */
#define DEFAULT_PRE_RANGE_VCSEL_PERIOD 14
#define DEFAULT_FINAL_RANGE_VCSEL_PERIOD 10

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

uint8_t VL53L0X_WriteReg8(uint8_t i2cDeviceAddress, uint8_t reg, uint8_t val) {
    uint8_t data[2];
    data[0] = reg;
    data[1] = val;

    if(HAL_I2C_Master_Transmit(&hi2c1, i2cDeviceAddress<<1, data, 2, HAL_MAX_DELAY) != HAL_OK) {
        return 1; // Error
    }
    return 0; // Success
}

/* I2C Write function for 16-bit register */
uint8_t VL53L0X_WriteReg16(uint8_t i2cDeviceAddress, uint8_t reg, uint16_t val) {
    uint8_t data[3];
    data[0] = reg;
    data[1] = (val >> 8) & 0xFF;
    data[2] = val & 0xFF;

    if(HAL_I2C_Master_Transmit(&hi2c1, i2cDeviceAddress<<1, data, 3, HAL_MAX_DELAY) != HAL_OK) {
        return 1;
    }
    return 0;
}

uint8_t VL53L0X_WriteReg32(uint8_t i2cDeviceAddress, uint8_t reg, uint32_t val) {
    uint8_t data[5];
    data[0] = reg;
    data[1] = (val >> 24) & 0xFF;
    data[2] = (val >> 16) & 0xFF;
    data[3] = (val >> 8) & 0xFF;
    data[4] = val & 0xFF;

    if(HAL_I2C_Master_Transmit(&hi2c1, i2cDeviceAddress<<1, data, 5, HAL_MAX_DELAY) != HAL_OK) {
        return 1;
    }
    return 0;
}

/* I2C Read function for 8-bit register */
uint8_t VL53L0X_ReadReg8(uint8_t i2cDeviceAddress, uint8_t reg, uint8_t *valP) {
    if(HAL_I2C_Master_Transmit(&hi2c1, i2cDeviceAddress<<1, &reg, 1, HAL_MAX_DELAY) != HAL_OK) {
        return 1;
    }
    if(HAL_I2C_Master_Receive(&hi2c1, i2cDeviceAddress<<1, valP, 1, HAL_MAX_DELAY) != HAL_OK) {
        return 1;
    }
    return 0;
}

/* I2C Read function for 16-bit register */
uint8_t VL53L0X_ReadReg16(uint8_t i2cDeviceAddress, uint8_t reg, uint16_t *valP) {
    uint8_t data[2];

    if(HAL_I2C_Master_Transmit(&hi2c1, i2cDeviceAddress<<1, &reg, 1, HAL_MAX_DELAY) != HAL_OK) {
        return 1;
    }
    if(HAL_I2C_Master_Receive(&hi2c1, i2cDeviceAddress<<1, data, 2, HAL_MAX_DELAY) != HAL_OK) {
        return 1;
    }

    *valP = ((uint16_t)data[0] << 8) | data[1];
    return 0;
}

/* Initialize VL53L0X device */
uint8_t VL53L0X_InitDevice(uint8_t i2cDeviceAddress) {
    uint8_t res;

    // Device initialization and configuration
    res = VL53L0X_WriteReg8(i2cDeviceAddress, SYSRANGE_START, 0x01);    // Start single range measurement
    HAL_Delay(1);

    // Configure interrupt
    res = VL53L0X_WriteReg8(i2cDeviceAddress, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);    // Enable interrupt on new data
    res = VL53L0X_WriteReg8(i2cDeviceAddress, GPIO_HV_MUX_ACTIVE_HIGH, 0x10);         // Set GPIO high when sample complete

    // Set default timing budget (can be adjusted based on requirements)
    res = VL53L0X_WriteReg16(i2cDeviceAddress, SYSTEM_INTERMEASUREMENT_PERIOD, 0x0100); // 256ms default

    return res;
}

/* Start continuous ranging */
uint8_t VL53L0X_StartContinuous(uint8_t i2cDeviceAddress, uint32_t period_ms) {
    uint8_t res;

    // Configure continuous mode
    res = VL53L0X_WriteReg8(i2cDeviceAddress, SYSRANGE_START, 0x02); // Start continuous mode

    // Set inter-measurement period
    uint16_t osc_calibrate_val;
    res = VL53L0X_ReadReg16(i2cDeviceAddress, 0xF8, &osc_calibrate_val);

    if (osc_calibrate_val != 0) {
        period_ms = (period_ms * osc_calibrate_val) / 1000;
    }

    res = VL53L0X_WriteReg32(i2cDeviceAddress, SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);

    return res;
}

/* Read range value in continuous mode */
uint8_t VL53L0X_ReadRangeContinuous(uint8_t i2cDeviceAddress, uint16_t *valP) {
    uint8_t res;
    uint8_t ready;
    uint16_t timeout = 500; // 500ms timeout

    do {
        res = VL53L0X_ReadReg8(i2cDeviceAddress, RESULT_INTERRUPT_STATUS, &ready);
        if (res != 0) {
            return res;
        }
        if (ready & 0x07) {
            break;
        }
        HAL_Delay(1);
        timeout--;
    } while (timeout > 0);

    if (timeout == 0) {
        return 1; // Timeout error
    }

    // Read range value
    uint16_t range;
    res = VL53L0X_ReadReg16(i2cDeviceAddress, RESULT_RANGE_STATUS + 10, &range); // Offset of 10 to get the range value
    *valP = range;

    // Clear interrupt
    res = VL53L0X_WriteReg8(i2cDeviceAddress, SYSTEM_INTERRUPT_CLEAR, 0x01);

    return res;
}

uint8_t ConfigureSensorTiming(uint8_t i2cDeviceAddress) {
    uint8_t status;

    // Switch to page 0
    status = VL53L0X_WriteReg8(i2cDeviceAddress, 0xFF, 0x00);

    // Set measurement timing budget
    // Setting to long range mode for more accurate readings
    status = VL53L0X_WriteReg8(i2cDeviceAddress, 0x01, 0xFF);  // Set long range mode

    // Set VCSEL period for pre-range
    status = VL53L0X_WriteReg8(i2cDeviceAddress, 0x50, 0x18);  // 18 = longer period

    // Set VCSEL period for final range
    status = VL53L0X_WriteReg8(i2cDeviceAddress, 0x52, 0x14);  // 14 = longer period

    // Increase timing budget for more accuracy
    status = VL53L0X_WriteReg16(i2cDeviceAddress, 0x46, 0x1000);  // Higher timing budget

    // Set signal rate limit (lower = more sensitive but more noise)
    status = VL53L0X_WriteReg16(i2cDeviceAddress, 0x44, 0x0028);  // 0.44 MCPS

    // Set measurement timing budget (inter-measurement period)
    status = VL53L0X_WriteReg16(i2cDeviceAddress, 0x40, 0x0100);  // ~100ms

    return status;
}

uint8_t CheckAndInitSensor(uint8_t currentAddress, uint8_t newAddress, GPIO_TypeDef* port, uint16_t xshut_pin, uint8_t sensorNum)
{
    uint8_t status;
    uint8_t val;

    // Enable only this sensor's XSHUT pin
    HAL_GPIO_WritePin(port, xshut_pin, GPIO_PIN_SET);
    HAL_Delay(10);  // Wait for sensor to wake up

    sprintf(uart_buf, "Checking Sensor %d...\r\n", sensorNum);
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);

    // Try reading from the new address first
    status = VL53L0X_ReadReg8(newAddress, 0x00, &val);

    if(status == 0) {
        // Sensor is already configured to new address
        sprintf(uart_buf, "Sensor %d already configured at address 0x%02X\r\n", sensorNum, newAddress);
        HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);

        // Initialize sensor with existing address
        status = VL53L0X_InitDevice(newAddress);
        if(status != 0) {
            sprintf(uart_buf, "Sensor %d init failed at address 0x%02X\r\n", sensorNum, newAddress);
            HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
            return 1;
        }

        status = ConfigureSensorTiming(newAddress);
        if(status != 0) {
            sprintf(uart_buf, "Failed to configure timing for Sensor %d\r\n", sensorNum);
            HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
            return 1;
        }

    } else {
        // Sensor needs to be configured with new address
        sprintf(uart_buf, "Configuring Sensor %d with new address...\r\n", sensorNum);
        HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);

        // Initialize with default address
        status = VL53L0X_InitDevice(currentAddress);
        if(status != 0) {
            sprintf(uart_buf, "Sensor %d init failed at default address\r\n", sensorNum);
            HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
            return 1;
        }

        status = ConfigureSensorTiming(currentAddress);
        if(status != 0) {
            sprintf(uart_buf, "Failed to configure timing for Sensor %d\r\n", sensorNum);
            HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
            return 1;
        }

        // Change to new address
        status = VL53L0X_WriteReg8(currentAddress, I2C_SLAVE_DEVICE_ADDRESS, newAddress);
        if(status != 0) {
            sprintf(uart_buf, "Failed to change Sensor %d address to 0x%02X\r\n", sensorNum, newAddress);
            HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
            return 1;
        }

        sprintf(uart_buf, "Sensor %d successfully configured to address 0x%02X\r\n", sensorNum, newAddress);
        HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
    }

    // Start continuous mode
    status = VL53L0X_StartContinuous(newAddress, 33);
    if(status != 0) {
        sprintf(uart_buf, "Failed to start continuous mode on Sensor %d\r\n", sensorNum);
        HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
        return 1;
    }

    return 0;
}

uint8_t VL53L0X_SetOffset(uint8_t i2cDeviceAddress, int8_t offset_mm)
{
    uint8_t status;

    // Write to offset register (signed byte value)
    status = VL53L0X_WriteReg8(i2cDeviceAddress,
                              ALGO_PART_TO_PART_RANGE_OFFSET_MM,
                              (uint8_t)offset_mm);

    // Enable offset compensation
    if(status == 0)
    {
        status = VL53L0X_WriteReg8(i2cDeviceAddress,
                                  SYSRANGE_PART_TO_PART_RANGE_OFFSET,
                                  0x01);
    }

    return status;
}
