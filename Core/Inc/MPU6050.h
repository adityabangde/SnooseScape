#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"


void MPU6050_init(void); //Initialize the MPU 
void MPU6050_Read_Accel (float *Ax, float *Ay, float *Az); //Read MPU Accelerator 
void MPU6050_Read_Gyro (float *Gx, float *Gy, float *Gz); //Read MPU Gyroscope
float moving_average(float *filter, float new_value);
void get_roll_pitch(int* roll, int* pitch);

#endif // MPU6050_H
