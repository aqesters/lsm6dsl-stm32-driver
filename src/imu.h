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
#define REG_GYRO_X		0x22
#define REG_GYRO_Y		0x24
#define REG_GYRO_Z		0x26

// Device modes
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
} IMU_Mode;

// Properties to track for device
typedef struct {
	I2C_HandleTypeDef* pointer_i2c_handle;
	IMU_Mode xl_rate;
	IMU_Mode gyro_rate;
	bool xl_lowpower;
	bool gyro_lowpower;
	float xl_sens;
	float gyro_sens;
	uint32_t i2c_timeout;
	bool connected;
} IMU;

// Main functions
IMU* IMU_Init(I2C_HandleTypeDef *hi2c, uint32_t i2c_timeout);
bool IMU_IsConnected(IMU *sensor);
HAL_StatusTypeDef XL_SetSamplingRate(IMU *sensor, IMU_Mode rate);
HAL_StatusTypeDef XL_LowPower_Enable(IMU *sensor);
HAL_StatusTypeDef XL_LowPower_Disable(IMU *sensor);
float XL_AccelX(const IMU *sensor);
float XL_AccelY(const IMU *sensor);
float XL_AccelZ(const IMU *sensor);
HAL_StatusTypeDef Gyro_SetSamplingRate(IMU *sensor, IMU_Mode rate);
HAL_StatusTypeDef Gyro_LowPower_Enable(IMU *sensor);
HAL_StatusTypeDef Gyro_LowPower_Disable(IMU *sensor);
float Gyro_AngularRateX(const IMU *sensor);
float Gyro_AngularRateY(const IMU *sensor);
float Gyro_AngularRateZ(const IMU *sensor);
HAL_StatusTypeDef IMU_Reset(IMU *sensor);
// TBD: Set XL full-scale
// TBD: Set Gyro full-scale

// Helper functions
HAL_StatusTypeDef I2C_SetBit(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint8_t reg_addr,
		uint8_t bit_pos, bool enable, uint32_t timeout);
int16_t ReadTwoBytes(const IMU *sensor, uint8_t reg, uint32_t timeout);

#endif /* INC_IMU_H_ */
