/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "servo.h"
#include "buzzer.h"
#include "MPU6050.h"
#include "ds3231.h"
#include "ssd1306.h"
#include "fonts.h"
#include <stdlib.h>
#include <math.h>
#include "interrupt.h"
#include "sensor_array.h"

#define TARGET_SPEED_RPM 150  // Target speed in RPM
#define COUNTS_PER_REV 20    // Encoder counts per revolution
#define PID_SAMPLE_TIME_MS 50
// Motor compensation adjustments
#define MOTOR1_COMPENSATION 1.0f    // Left motor baseline
#define MOTOR2_COMPENSATION 1.15f   // Slightly increased for right motor

#define KP 2.0f
#define KI 0.05f
#define KD 0.4f

// Anti-windup and smoothing
#define MAX_INTEGRAL 60.0f  // Increased to allow more correction
#define OUTPUT_RAMP_RATE 15.0f  // Increased for faster response
#define ALPHA 0.6f  // Reduced from 0.7 for less aggressive filtering

#define SAFETY_DISTANCE_CM 20        // Minimum safe distance
//#define TURN_SPEED 50               // Speed for turning
#define NORMAL_SPEED 70             // Normal forward speed
#define MOTOR_FORWARD_OFFSET 100
#define MOTOR_BACKWARD_OFFSET 15
#define MOTOR_SPEED_OFFSET 15

#define TURN_SPEED_MAX 100           // Maximum turning speed

#define TURN_ACTIVE_TIME  30    // Time to turn (ms)
#define TURN_PAUSE_TIME   400    // Time to pause (ms)

/* Private variables */
typedef struct {
    float target_rpm;
    float current_rpm;
    float error;
    float last_error;
    float integral;
    float derivative;
    float output;
    float filtered_output;
    float last_output;
    uint32_t last_time;
    uint32_t counts;
    uint32_t last_counts;
} PID_TypeDef;

PID_TypeDef pid_motor1;
PID_TypeDef pid_motor2;
uint8_t current_direction = 0;  // 0=stop, 1=forward, 2=backward, 3=left, 4=right

uint32_t sensor1_count = 0;
uint32_t sensor2_count = 0;
uint8_t last_state_pc2 = 0;
uint8_t last_state_pc3 = 0;
uint32_t last_print_time = 0;

// Define turn directions
#define LEFT  0
#define RIGHT 1
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define RX_BUFFER_SIZE 100
char timerBuffer[RX_BUFFER_SIZE];

int roll , pitch = 0;

int timer_type, time_recieved = 0;
uint8_t hour, min, month, day, year;
uint8_t alarmVar = 0;
uint8_t alarmHour, alarmMin = 0;

uint8_t UART1_rxBuffer[RX_BUFFER_SIZE];
uint8_t rxData[1];  // Buffer for single byte reception
uint16_t rxIndex = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint32_t timerPreviousValue = 0;
uint32_t previousAlarmMin = 0;
uint32_t pidPreviousValue = 0;
const uint32_t INTERVAL = 1000;    // Print every 1 second
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t stop_motors = 0;

/* Function Prototypes */
void check_sensors(void);
void PID_Init(PID_TypeDef *pid, float target_rpm);

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* Definitions for timeTask */
osThreadId_t timeTaskHandle;
const osThreadAttr_t timeTask_attributes = {
  .name = "timeTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for pidTask */
osThreadId_t pidTaskHandle;
const osThreadAttr_t pidTask_attributes = {
  .name = "pidTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for lidarTask */
osThreadId_t lidarTaskHandle;
const osThreadAttr_t lidarTask_attributes = {
  .name = "lidarTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
void TimeTask(void *argument);
void PidTask(void *argument);
void LidarTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        // Store received byte in buffer
		UART1_rxBuffer[rxIndex++] = rxData[0];
		// If buffer is full or we received a newline character
		if(rxIndex >= RX_BUFFER_SIZE || rxData[0] == '\n' || rxData[0] == '\r')
		{
			// Echo back the received data
			processTimeData(UART1_rxBuffer, &hour, &min, &day, &month, &year, &timer_type);
			HAL_UART_Transmit(&huart2, UART1_rxBuffer, rxIndex, 100);
			rxIndex = 0;  // Reset buffer
			time_recieved = 1;
		}
        // Restart reception for next byte
        HAL_UART_Receive_IT(&huart2, rxData, 1);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_7) // Changed to PA9
     {
         alarmVar = 0;
         stop_motors = 0;
         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
     }
}

uint8_t isValidObstacle(uint16_t distance) {
    // Valid obstacle detection range
    if(distance < SAFETY_DISTANCE_CM) {
        return 1;  // Obstacle detected
    }
    return 0;  // No obstacle
}

void set_motor_speed(uint8_t motor, int16_t speed)
{
    uint16_t pwm_value;

    // Limit speed to -199 to +199 for 5kHz PWM
    if(speed > 199) speed = 199;
    if(speed < -199) speed = -199;

    // Get absolute PWM value
    pwm_value = abs(speed);

    if(motor == 1)  // Left motor (PA5, PA6)
    {
        if(speed >= 0)
        {
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_value);  // PA5
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);         // PA6
        }
        else
        {
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_value);
        }
    }
    else if(motor == 2)  // Right motor (PA7, PB6)
    {
        if(speed >= 0)
        {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm_value);  // PA7
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);         // PB6
        }
        else
        {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm_value);
        }
    }
}

void motors_forward(uint8_t speed)
{
    // Convert speed percentage to target RPM
    float target_rpm = (float)speed * TARGET_SPEED_RPM / 100.0f;

    // Set same target RPM but compensation will be applied in PID calculation
    pid_motor1.target_rpm = target_rpm;
    pid_motor2.target_rpm = target_rpm;
    current_direction = 1;
}

void motors_backward(uint8_t speed)
{
    float target_rpm = (float)speed * TARGET_SPEED_RPM / 100.0f;

    // For backward motion, use negative RPM targets
    pid_motor1.target_rpm = -target_rpm;
    pid_motor2.target_rpm = -target_rpm;
    current_direction = 2;
}

void motors_left(uint8_t speed)
{
    float target_rpm = (float)speed * TARGET_SPEED_RPM / 100.0f;

    // For turning, motors run in opposite directions
    pid_motor1.target_rpm = target_rpm;
    pid_motor2.target_rpm = -target_rpm;
    current_direction = 3;
}

void motors_right(uint8_t speed)
{
    float target_rpm = (float)speed * TARGET_SPEED_RPM / 100.0f;

    pid_motor1.target_rpm = -target_rpm;
    pid_motor2.target_rpm = target_rpm;
    current_direction = 4;
}

void motors_stop(void)
{
    pid_motor1.target_rpm = 0;
    pid_motor2.target_rpm = 0;
    current_direction = 0;

    // Immediately stop motors
    set_motor_speed(1, 0);
    set_motor_speed(2, 0);

    // Reset PID controllers
    pid_motor1.integral = 0;
    pid_motor2.integral = 0;
}

/* Calculate RPM from sensor counts */
float Calculate_RPM(uint32_t counts, uint32_t last_counts, uint32_t time_diff_ms) {
    uint32_t count_diff = counts - last_counts;
    // Convert to RPM: (counts/ticks_per_rev) * (60000/time_diff_ms)
    return ((float)count_diff * 60000.0f) / (COUNTS_PER_REV * time_diff_ms);
}

float PID_Calculate(PID_TypeDef *pid, uint32_t current_counts, uint8_t motor_number) {
    uint32_t current_time = HAL_GetTick();
    uint32_t time_diff = current_time - pid->last_time;
    float new_output = pid->filtered_output;  // Initialize with current value

    if (time_diff >= PID_SAMPLE_TIME_MS) {
        // Calculate current speed from sensor counts
        pid->current_rpm = Calculate_RPM(current_counts, pid->last_counts, time_diff);

        // Apply motor compensation
        float compensation = (motor_number == 1) ? MOTOR1_COMPENSATION : MOTOR2_COMPENSATION;

        // Calculate error with compensation
        pid->error = (pid->target_rpm - pid->current_rpm) * compensation;

        // More aggressive integral for low speeds
        float ki_adaptive = (pid->current_rpm < pid->target_rpm/2) ? KI * 1.5f : KI;

        // Calculate integral with anti-windup
        if (fabsf(pid->output) < 100.0f) {
            pid->integral += pid->error * ((float)time_diff / 1000.0f);
            pid->integral = fminf(fmaxf(pid->integral, -MAX_INTEGRAL), MAX_INTEGRAL);
        }

        // Derivative with filtering
        pid->derivative = (pid->error - pid->last_error) / ((float)time_diff / 1000.0f);
        pid->derivative = (ALPHA * pid->derivative) + ((1.0f - ALPHA) * pid->derivative);

        // Calculate PID output
        new_output = (KP * pid->error) +
                    (ki_adaptive * pid->integral) +
                    (KD * pid->derivative);

        // Smoother output changes
        float output_delta = new_output - pid->last_output;
        if (fabsf(output_delta) > OUTPUT_RAMP_RATE) {
            new_output = pid->last_output + (output_delta > 0 ? OUTPUT_RAMP_RATE : -OUTPUT_RAMP_RATE);
        }

        // Store values for next iteration
        pid->last_error = pid->error;
        pid->last_output = new_output;
        pid->last_time = current_time;
        pid->last_counts = current_counts;
        pid->filtered_output = (ALPHA * new_output) + ((1.0f - ALPHA) * pid->filtered_output);

        // Debug output
        sprintf(uart_buf, "Motor%d RPM: %d.%d, Target: %d.%d, Out: %d.%d\r\n",
                motor_number,
                (int)pid->current_rpm, (int)(pid->current_rpm * 10) % 10,
                (int)pid->target_rpm, (int)(pid->target_rpm * 10) % 10,
                (int)new_output, (int)(new_output * 10) % 10);
        HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, strlen(uart_buf), 100);
    }

    return pid->filtered_output;
}

void update_motor_control(void)
{
    // Read speed sensors
    check_sensors();

    if(current_direction != 0) // If motors should be moving
    {
        // Calculate PID outputs
        float m1_output = PID_Calculate(&pid_motor1, sensor1_count, 1);
        float m2_output = PID_Calculate(&pid_motor2, sensor2_count, 2);

        // Convert PID output to PWM values
        int16_t pwm1, pwm2;

        // Handle direction
        if (current_direction == 1) { // Forward
            pwm1 = (int16_t)(m1_output * 1.99f);
            pwm2 = (int16_t)(m2_output * 1.99f);
        }
        else if (current_direction == 2) { // Backward
            pwm1 = -(int16_t)(m1_output * 1.99f);
            pwm2 = -(int16_t)(m2_output * 1.99f);
        }

        // Apply to motors
        set_motor_speed(1, pwm1);
        set_motor_speed(2, pwm2);

        // Debug
        sprintf(uart_buf, "PWM1: %d, PWM2: %d\r\n", pwm1, pwm2);
        HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), 100);
    }
}

void check_sensors(void) {
    // Read current states
    uint8_t current_state_pc2 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);
    uint8_t current_state_pc3 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3);

    // Check for rising edge on PC2 (Motor 1 sensor)
    if(current_state_pc2 == 1 && last_state_pc2 == 0) {
        sensor1_count++;
    }

    // Check for rising edge on PC3 (Motor 2 sensor)
    if(current_state_pc3 == 1 && last_state_pc3 == 0) {
        sensor2_count++;
    }

    // Update last states
    last_state_pc2 = current_state_pc2;
    last_state_pc3 = current_state_pc3;
}

/* PID initialization */
void PID_Init(PID_TypeDef *pid, float target_rpm) {
    pid->target_rpm = target_rpm;
    pid->current_rpm = 0;
    pid->error = 0;
    pid->last_error = 0;
    pid->integral = 0;
    pid->derivative = 0;
    pid->output = 0;
    pid->filtered_output = 0;
    pid->last_output = 0;
    pid->last_time = HAL_GetTick();
    pid->counts = 0;
    pid->last_counts = 0;
}

// In your navigation function:
void turn_with_steps(uint8_t direction, uint32_t currentTime) {
    static uint32_t cycleStartTime = 0;
    static uint8_t isNewCycle = 1;

    // Start new turn cycle
    if(isNewCycle) {
        cycleStartTime = currentTime;
        isNewCycle = 0;
    }

    // Calculate time within the current cycle
    uint32_t timeInCycle = currentTime - cycleStartTime;

    // Total cycle time
    const uint32_t TOTAL_CYCLE_TIME = TURN_ACTIVE_TIME + TURN_PAUSE_TIME;

    // If we've completed a cycle, start a new one
    if(timeInCycle >= TOTAL_CYCLE_TIME) {
        cycleStartTime = currentTime;
        timeInCycle = 0;
    }

    // During active turn time
    if(timeInCycle < TURN_ACTIVE_TIME) {
        if(direction == RIGHT) {
            motors_right(TURN_SPEED_MAX);
        } else {
            motors_left(TURN_SPEED_MAX);
        }
    }
    // During pause time
    else {
        motors_stop();
    }

    // Debug output
    sprintf(uart_buf, "Turn Cycle: %lu ms, State: %s\r\n",
            timeInCycle,
            (timeInCycle < TURN_ACTIVE_TIME) ? "Turning" : "Stopped");
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), 100);
}

void navigateRobot(uint16_t *filteredDistances) {
    static uint32_t stateStartTime = 0;
    uint8_t leftObstacles = 0;
    uint8_t rightObstacles = 0;
    uint32_t currentTime = HAL_GetTick();

    // Count obstacles on left and right
    for(int i = 0; i < 2; i++) {
        if(isValidObstacle(filteredDistances[i])) {
            leftObstacles++;
        }
    }
    for(int i = 3; i < 5; i++) {
        if(isValidObstacle(filteredDistances[i])) {
            rightObstacles++;
        }
    }

    // If obstacles on left side, turn right gradually
    // Handle center sensor separately if needed
    if((isValidObstacle(filteredDistances[2]) && filteredDistances[2] < SAFETY_DISTANCE_CM/2)
    	|| (isValidObstacle(filteredDistances[0]) && filteredDistances[0] < SAFETY_DISTANCE_CM/2)
		|| (isValidObstacle(filteredDistances[4]) && filteredDistances[4] < SAFETY_DISTANCE_CM/2)) {
        // Consider center obstacle based on context
    	if(currentTime - stateStartTime > 300) {
    		motors_backward(NORMAL_SPEED);
    	 stateStartTime = currentTime;
        }

    }
    else if(leftObstacles > rightObstacles) {
    	if(isValidObstacle(filteredDistances[0]) || isValidObstacle(filteredDistances[1])|| isValidObstacle(filteredDistances[2])) {
    		 turn_with_steps(RIGHT, currentTime);
        }
//        sprintf(uart_buf, "Left obstacle detected, turning RIGHT\r\n");
    }
    // If obstacles on right side, turn left gradually
    else if(leftObstacles < rightObstacles) {
    	if(isValidObstacle(filteredDistances[2]) || isValidObstacle(filteredDistances[3]) || isValidObstacle(filteredDistances[4])) {
            // Implement stepped turning
    		 turn_with_steps(LEFT, currentTime);
        }
//        sprintf(uart_buf, "Right obstacle detected, turning LEFT\r\n");
    }

    else if(leftObstacles > 0 && leftObstacles == rightObstacles) {
    	// Implement stepped turning
//        sprintf(uart_buf, " BLOCKING detected, turning LEFT\r\n");
    	 turn_with_steps(LEFT, currentTime);
    }
    else if(leftObstacles > rightObstacles && leftObstacles > 1) {  // Need at least 2 sensors to trigger
        // Turn right
		if(isValidObstacle(filteredDistances[0]) || isValidObstacle(filteredDistances[1])|| isValidObstacle(filteredDistances[2])) {
			 turn_with_steps(RIGHT, currentTime);
		}
//		sprintf(uart_buf, "Left obstacle detected, turning RIGHT\r\n");
    }
    else if(rightObstacles > leftObstacles && rightObstacles > 1) {  // Need at least 2 sensors to trigger
        // Turn left
    	if(isValidObstacle(filteredDistances[2]) || isValidObstacle(filteredDistances[3]) || isValidObstacle(filteredDistances[4])) {
            // Implement stepped turning
    		 turn_with_steps(LEFT, currentTime);
        }
//        sprintf(uart_buf, "Right obstacle detected, turning LEFT\r\n");
    }

    // If path is clear, go forward and reset turn speed
    else {
        motors_forward(NORMAL_SPEED);
    }

//    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), 100);

//    // Debug output
//    sprintf(uart_buf, "Distances: %d,%d,%d,%d,%d\r\n",
//            filteredDistances[0], filteredDistances[1],
//            filteredDistances[2], filteredDistances[3],
//            filteredDistances[4]);
//    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), 100);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  Debug_Print("\r\n[DEBUG] System Initialized\r\n");
  HAL_UART_Receive_IT(&huart2, rxData, 1);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of timeTask */
  timeTaskHandle = osThreadNew(TimeTask, NULL, &timeTask_attributes);

  /* creation of pidTask */
  pidTaskHandle = osThreadNew(PidTask, NULL, &pidTask_attributes);

  /* creation of lidarTask */
  lidarTaskHandle = osThreadNew(LidarTask, NULL, &lidarTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00F12981;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x10D19CE4;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 399;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 3999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 79;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 79;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 382;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC5 PC6 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_TimeTask */
/**
  * @brief  Function implementing the timeTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_TimeTask */
void TimeTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	static uint32_t led_last_time = 0;
	static uint8_t led_state = 0;  // 0 = OFF cycle, 1 = ON cycle
	MPU6050_init();
	ssd1306_Init(&hi2c3);
	buzzer_init();
	start_mario_melody();  // Start with main theme

	Set_init_time( __DATE__, __TIME__);

	servo_init(&htim1, TIM_CHANNEL_2);
	setServoAngle(&htim1, TIM_CHANNEL_2, 0);
	HAL_Delay(1000);
  /* Infinite loop */
  for(;;)
  {
	uint32_t current_time = HAL_GetTick();
    if(time_recieved == 1){
    	if(timer_type == 0){
    		Set_Time(0, min, hour, 1, day, month, year);
    	}
    	if(timer_type == 1){
    		alarmHour = hour;
    		alarmMin = min;
    		snprintf(timerBuffer, sizeof(timerBuffer), "A:%02d:%02d", alarmHour, alarmMin);
    		ssd1306_SetCursor(0, 40); // column, row
    		ssd1306_WriteString(timerBuffer, Font_11x18, White);
    	}
      time_recieved = 0;
    }

	Get_Time();
	get_roll_pitch(&roll, &pitch);

	snprintf(timerBuffer, sizeof(timerBuffer), "T:%02d:%02d:%02d", time.hour, time.minutes, time.seconds);
	ssd1306_SetCursor(0, 0); // column, row
	ssd1306_WriteString(timerBuffer, Font_11x18, White);

	sprintf(timerBuffer, "D:%02d/%02d/%02d",time.dayofmonth, time.month, time.year);
	ssd1306_SetCursor(0, 20); // column, row
	ssd1306_WriteString(timerBuffer, Font_11x18, White);
	ssd1306_UpdateScreen(&hi2c3); // Copy all data from local screen buffer to the screen

//		snprintf(timerBuffer, sizeof(timerBuffer), "time: %02d:%02d:%02d\r\n", time.hour, time.minutes, time.seconds);
//		HAL_UART_Transmit(&huart2, (uint8_t *)timerBuffer, strlen(timerBuffer), HAL_MAX_DELAY);
//
//		snprintf(timerBuffer, sizeof(timerBuffer), "Roll: %d, Pitch: %d\r\n", roll, pitch);
//		HAL_UART_Transmit(&huart2, (uint8_t *)timerBuffer, strlen(timerBuffer), HAL_MAX_DELAY);

	if(alarmHour == time.hour && alarmMin == time.minutes && (previousAlarmMin != time.minutes)){
		previousAlarmMin = time.minutes;
		alarmVar = 1;
	}
	else if(previousAlarmMin != time.minutes){
		previousAlarmMin = 0;
	}

	if(alarmVar == 1){
		stop_motors = 1;
		if(led_state == 0)  // During OFF cycle
		    {
		        if(current_time - led_last_time >= 1000)  // Check if 1 second passed
		        {
		            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);  // Turn ON
		            led_last_time = current_time;
		            led_state = 1;  // Switch to ON cycle
		        }
		    }
		 else  // During ON cycle
		    {
		        if(current_time - led_last_time >= 1000)  // Check if 1 second passed
		        {
		            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);  // Turn OFF
		            led_last_time = current_time;
		            led_state = 0;  // Switch to OFF cycle
		        }
		    }

	}


	if(roll>90 || roll<-90 || pitch>90 || pitch<-90 ){
		setServoAngle(&htim1, TIM_CHANNEL_2, 180);
	}
	else {
		setServoAngle(&htim1, TIM_CHANNEL_2, 0);
	}
	osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_PidTask */
/**
* @brief Function implementing the pidTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PidTask */
void PidTask(void *argument)
{
  /* USER CODE BEGIN PidTask */
  /* Infinite loop */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

  PID_Init(&pid_motor1, TARGET_SPEED_RPM);
  PID_Init(&pid_motor2, TARGET_SPEED_RPM);

  for(;;)
  {
	if(stop_motors !=0 ){
		navigateRobot(filteredDistances);
		// Small delay to prevent overwhelming the system
	}
	else{
		motors_stop();
	}
	 // Update PID control
	update_motor_control();

	// Debug output
//	static uint32_t last_debug = 0;
//	if (HAL_GetTick() - last_debug >= 1000) {
//	  sprintf(uart_buf, "Dir: %d, M1 RPM: %d, M2 RPM: %d\r\n",
//			  current_direction,
//			  (int)pid_motor1.current_rpm,
//			  (int)pid_motor2.current_rpm);
//	  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), 100);
//	  last_debug = HAL_GetTick();
//	}

	osDelay(1);
  }
  /* USER CODE END PidTask */
}

/* USER CODE BEGIN Header_LidarTask */
/**
* @brief Function implementing the lidarTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LidarTask */
void LidarTask(void *argument)
{
  /* USER CODE BEGIN LidarTask */
  /* Infinite loop */
	sprintf(uart_buf, "Starting VL53L0X multiple sensor initialization...\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);

	// Initially set all XSHUT pins low
	HAL_GPIO_WritePin(SENSOR_GPIO_PORT_C, SENSOR1_XSHUT_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SENSOR_GPIO_PORT_A, SENSOR2_XSHUT_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SENSOR_GPIO_PORT_C, SENSOR3_XSHUT_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SENSOR_GPIO_PORT_C, SENSOR4_XSHUT_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SENSOR_GPIO_PORT_C, SENSOR5_XSHUT_PIN, GPIO_PIN_RESET);
	HAL_Delay(10);

	// Initialize sensors one by one, keeping previous ones enabled
	CheckAndInitSensor(0x29, 0x30, SENSOR_GPIO_PORT_C, SENSOR1_XSHUT_PIN, 1);
	// First sensor stays enabled, initialize second
	CheckAndInitSensor(0x29, 0x31, SENSOR_GPIO_PORT_A, SENSOR2_XSHUT_PIN, 2);
	// First two sensors stay enabled, initialize third
	CheckAndInitSensor(0x29, 0x32, SENSOR_GPIO_PORT_C, SENSOR3_XSHUT_PIN, 3);
	CheckAndInitSensor(0x29, 0x33, SENSOR_GPIO_PORT_C, SENSOR4_XSHUT_PIN, 4);
	CheckAndInitSensor(0x29, 0x34, SENSOR_GPIO_PORT_C, SENSOR5_XSHUT_PIN, 5);

	//    VL53L0X_SetOffset(0x30, -20);
	// Set +15mm offset for sensor 2
	VL53L0X_SetOffset(0x31, 0.3);
	//    VL53L0X_SetOffset(0x32, 15);
	VL53L0X_SetOffset(0x33, 0.3);
	VL53L0X_SetOffset(0x34, -1);

	uint8_t status;
	uint16_t distance1, distance2, distance3, distance4, distance5;

	while(1)
	{
		uint32_t sum1 = 0, sum2 = 0, sum3 = 0, sum4 = 0, sum5 = 0;
		uint16_t filtered_dist1, filtered_dist2, filtered_dist3, filtered_dist4, filtered_dist5;

		// Read and store in circular buffer
		status = VL53L0X_ReadRangeContinuous(0x30, &distance1);
		HAL_Delay(1);
		if(status != 0) {
		  distance1 = 1000;  // Invalid reading due to error
		}
		else if(distance1 > 1000) {
		  distance1 = 1000;  // Limit maximum distance to 500
		}
		buffer1[buffer_index] = distance1;

		status = VL53L0X_ReadRangeContinuous(0x31, &distance2);
		HAL_Delay(1);
		if(status != 0) {
		  distance2 = 1000;  // Invalid reading due to error
		}
		else if(distance2 > 1000) {
		  distance2 = 1000;  // Limit maximum distance to 500
		}
		buffer2[buffer_index] = distance2;

		status = VL53L0X_ReadRangeContinuous(0x32, &distance3);
		HAL_Delay(1);
		if(status != 0) {
		  distance3 = 1000;  // Invalid reading due to error
		}
		else if(distance3 > 1000) {
		  distance3 = 1000;  // Limit maximum distance to 500
		}
		buffer3[buffer_index] = distance3;

		status = VL53L0X_ReadRangeContinuous(0x33, &distance4);
		HAL_Delay(1);
		if(status != 0) {
		  distance4 = 1000;  // Invalid reading due to error
		}
		else if(distance4 > 1000) {
		  distance4 = 1000;  // Limit maximum distance to 500
		}
		buffer4[buffer_index] = distance4;

		status = VL53L0X_ReadRangeContinuous(0x34, &distance5);
		HAL_Delay(1);
		if(status != 0) {
		  distance5 = 1000;  // Invalid reading due to error
		}
		else if(distance5 > 1000) {
		  distance5 = 1000;  // Limit maximum distance to 500
		}
		buffer5[buffer_index] = distance5;

		// Calculate averages
		for(int i = 0; i < FILTER_SIZE; i++) {
			sum1 += buffer1[i];
			sum2 += buffer2[i];
			sum3 += buffer3[i];
			sum4 += buffer4[i];
			sum5 += buffer5[i];
		}

		filtered_dist1 = sum1 / FILTER_SIZE;
		filtered_dist2 = sum2 / FILTER_SIZE;
		filtered_dist3 = sum3 / FILTER_SIZE;
		filtered_dist4 = sum4 / FILTER_SIZE;
		filtered_dist5 = sum5 / FILTER_SIZE;

		// Update buffer index
		buffer_index = (buffer_index + 1) % FILTER_SIZE;

		//	//	      	     Output filtered distances in cm
		//	sprintf(uart_buf, "%d ---- %d ---- %d ---- %d ---- %d\r\n",
		//			filtered_dist1/10, filtered_dist2/10, filtered_dist3/10, filtered_dist4/10, filtered_dist5/10);
		//	HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);

		filteredDistances[0] = filtered_dist1/10;
		filteredDistances[1] = filtered_dist2/10;
		filteredDistances[2] = filtered_dist3/10;
		filteredDistances[3] = filtered_dist4/10;
		filteredDistances[4] = filtered_dist5/10;
		osDelay(1);
	}
  /* USER CODE END LidarTask */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
