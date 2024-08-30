#include <main.h>
#include <mpu9250.h>
#include <uart.h> // Incluye tus funciones personalizadas de UART
#include <string.h>
// Definiciones de registros basadas en el mapa de registros y la especificación del producto
#define WHO_AM_I_REG 0x71
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
#define INT_PIN_CFG 0x37
#define AK8963_CNTL 0x0A
#define AK8963_ASAX 0x10

// Funciones básicas para interactuar con MPU9250


// Implementación simplificada de las funciones de lectura y calibración.

void MPU9250_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t data = 0x00;
    // Despertar el dispositivo configurando el bit de reset en el registro PWR_MGMT_1
    HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, PWR_MGMT_1, 1, &data, 1, 1000);

    // Configuraciones adicionales según sea necesario...
}

#include <mpu9250.h>
#include <uart.h> // Incluye tus funciones personalizadas de UART

// Implementación simplificada para enviar datos de acelerómetro via UART
void MPU9250_ReadAccel(I2C_HandleTypeDef *hi2c, float *accelData) {
    uint8_t rawData[6];
    HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS, ACCEL_XOUT_H, 1, rawData, 6, 1000);

    for (int i = 0; i < 3; ++i) {
        int16_t raw = (int16_t)(rawData[2 * i] << 8 | rawData[2 * i + 1]);
        accelData[i] = (float)raw * 2.0 / 32768.0;
    }
}

void MPU9250_ReadGyro(I2C_HandleTypeDef *hi2c, float *gyroData) {
    uint8_t rawData[6];
    HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS, GYRO_XOUT_H, 1, rawData, 6, 1000);

    for (int i = 0; i < 3; ++i) {
        int16_t raw = (int16_t)(rawData[2 * i] << 8 | rawData[2 * i + 1]);
        gyroData[i] = (float)raw * 250.0 / 32768.0;
    }
}

void MPU9250_ReadMag(I2C_HandleTypeDef *hi2c, float *magData) {
    uint8_t rawData[7];
    HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS, AK8963_CNTL, 1, rawData, 7, 1000);

    for (int i = 0; i < 3; ++i) {
        int16_t raw = (int16_t)(rawData[2 * i] | rawData[2 * i + 1] << 8);
        magData[i] = (float)raw;
    }
}


void MPU9250_Calibrate(I2C_HandleTypeDef *hi2c) {
    // Calibrar el MPU9250...
    // La calibración adecuada requiere un procedimiento específico y extenso que depende de su aplicación.
    // Aquí asumimos una calibración simple que podría incluir la lectura de valores en un estado estático y ajustar según sea necesario.
}
