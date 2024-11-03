/*
 * mpu9250.h
 *
 *  Created on: Apr 17, 2024
 *      Author: Alejo
 */

#ifndef MPU9250_H
#define MPU9250_H
#include <stdio.h>
#include "stm32f1xx_hal.h"
#include <main.h> // Asegúrate de que este archivo incluya tu configuración de UART_HandleTypeDef

extern char mensaje[500];
#define MPU9250_ADDRESS 0xD0

// Funciones para inicializar y leer datos del MPU9250
void MPU9250_Init(I2C_HandleTypeDef *hi2c);
// mpu9250.h

void MPU9250_ReadAccel(I2C_HandleTypeDef *hi2c, float *accelData);
void MPU9250_ReadGyro(I2C_HandleTypeDef *hi2c, float *gyroData);
void MPU9250_ReadMag(I2C_HandleTypeDef *hi2c, float *magData);
void MPU9250_Calibrate(I2C_HandleTypeDef *hi2c);

#endif
