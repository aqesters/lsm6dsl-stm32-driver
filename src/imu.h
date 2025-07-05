/*
 * imu.h
 *
 *  Created on: Jul 3, 2025
 *      Author: aesters
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

// Includes I2C, UART, and other HAL drivers
#include "stm32l4xx_hal.h"	// HAL drivers for the STM32L4 MCU
#include <stdbool.h>

// Device address
#define LSM6DSL_ADDRESS 	0b1101010
#define IMU_ADDRESS			(LSM6DSL_ADDRESS << 1)

// Register addresses
#define REG_ID				0x0F
#define REG_CTRL1			0x10
#define REG_CTRL2			0x11
#define REG_CTRL3			0x12
#define REG_CTRL6			0x15
#define REG_CTRL7			0x16
#define REG_XL_X			0x28
#define REG_XL_Y			0x2A
#define REG_XL_Z			0x2C
#define REG_GYRO_ROLL		0x22
#define REG_GYRO_PITCH		0x24
#define REG_GYRO_YAW		0x26

// Sampling rates
typedef enum {
	POWER_DOWN = 0,
	FREQ_1P6 = 0b1011,	// 1.6 Hz -> only available for xl_rate, and only if IMU.xl_lowpower = true !
	FREQ_12P5 = 1,		// 12.5 Hz
	FREQ_26,			// 26 Hz
	FREQ_52,			// 52 Hz
	FREQ_104,			// 104 Hz
	FREQ_208,			// 208 Hz
	FREQ_416,			// 416 Hz
	FREQ_833,			// 833 Hz
	FREQ_1660,			// 1.66 kHz
	FREQ_3330,			// 3.33 kHz
	FREQ_6660			// 6.66 kHz
} freq;

typedef struct {
	I2C_HandleTypeDef* pointer_i2c_handle;
	freq xl_rate;
	freq gyro_rate;
	bool xl_lowpower;
	bool gyro_lowpower;
	float xl_sens;
	float gyro_sens;
	uint32_t i2c_timeout;
	bool connected;
} IMU;

// Main functions
IMU IMU_Init(I2C_HandleTypeDef *hi2c, uint32_t i2c_timeout);
bool IMU_IsConnected(IMU sensor);
int XL_SetSamplingRate(IMU sensor, freq rate);
int XL_LowPower_Enable(IMU sensor);
int XL_LowPower_Disable(IMU sensor);
float XL_GetX(IMU sensor);
float XL_GetY(IMU sensor);
float XL_GetZ(IMU sensor);
int Gyro_SetSamplingRate(IMU sensor, freq rate);
int Gyro_LowPower_Enable(IMU sensor);
int Gyro_LowPower_Disable(IMU sensor);
float Gyro_GetRoll(IMU sensor);
float Gyro_GetPitch(IMU sensor);
float Gyro_GetYaw(IMU sensor);
int IMU_Reset(IMU sensor);
// TBD: Set XL full-scale
// TBD: Set Gyro full-scale

// Helper functions
int I2C_SetBit(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint8_t reg_addr,
		uint8_t bit_pos, bool enable, uint32_t timeout);
int16_t ReadTwoBytes(IMU sensor, uint8_t reg, uint32_t timeout);

#endif /* INC_IMU_H_ */
