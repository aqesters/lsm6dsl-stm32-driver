/*
* Example code for reading accelerometer and gyroscope data from an IMU sensor
* Prints results to a serial port using the UART HAL library.
* This code is designed for STM32L4 series microcontrollers.

* Target microcontroller: STM32L4 series
* Libraries used: HAL, IMU, UART, I2C
*/

// Includes
#include "imu.h"

// Variables and macros
#define I2C_TIMEOUT 1000
#define N_VALUES 6

// Check UART connection ("huart1" is the UART handler)
assert(HAL_UART_Transmit(&huart1, (uint8_t*) "UART ready!\r\n", 14, UART_TIMEOUT) == 0);

// Initialize IMU sensor using I2C handler ("hi2c2" in this case)
IMU sensor = IMU_Init(&hi2c2, I2C_TIMEOUT);
// -> By default, sampling rates are set to "POWER_DOWN" (disabled).
// -> By default, low-power modes are disabled.

// Configure accelerometer
XL_SetSamplingRate(sensor, FREQ_12P5);  // Set 12.5 Hz sampling rate
XL_LowPower_Enable(sensor);             // Enable low-power mode

// Configure gyroscope
Gyro_SetSamplingRate(sensor, FREQ_12P5);    // Set 12.5 Hz sampling rate
Gyro_LowPower_Enable(sensor);               // Enable low-power mode

// Run continuously
while (1)
{
    // Get all IMU measurements
    float ACCEL_X = XL_GetX(sensor);
    float ACCEL_Y = XL_GetY(sensor);
    float ACCEL_Z = XL_GetZ(sensor);
    float GYRO_R = Gyro_GetRoll(sensor);
    float GYRO_P = Gyro_GetPitch(sensor);
    float GYRO_Y = Gyro_GetYaw(sensor);

    // Send values to serial port
    float values[N_VALUES] = {ACCEL_X, ACCEL_Y, ACCEL_Z, GYRO_R, GYRO_P, GYRO_Y};
    char buffer[10] = {0};	// stores number as a string 

    for (int i = 0; i < N_VALUES; ++i)
    {
        sprintf(buffer, "%.2f, ", values[i]);
        HAL_UART_Transmit(&huart1, buffer, strlen(buffer), UART_TIMEOUT);
    }

    HAL_UART_Transmit(&huart1, (uint8_t*) "\r\n", 3, UART_TIMEOUT);
    HAL_Delay(200);
}