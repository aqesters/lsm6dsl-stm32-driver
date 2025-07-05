/*
 * imu.c
 *
 *  Created on: Jul 3, 2025
 *      Author: Ari Esters
 *
 *  C source code for LSM6DSL inertial measurement unit (IMU) sensor from STMicroelectronics
 *  Capable of linear acceleration and gyroscopic measurements
 *  Written for compatibility with STM32L4 HAL drivers
 */

#include "imu.h"

// Initialize and return data structure for IMU sensor
IMU IMU_Init(I2C_HandleTypeDef* pointer_i2c_handle, uint32_t timeout)
{
	// Default values (based on datasheet)
	IMU sensor;
	sensor.pointer_i2c_handle = pointer_i2c_handle;
	sensor.xl_rate = POWER_DOWN;
	sensor.gyro_rate = POWER_DOWN;
	sensor.xl_lowpower = false;
	sensor.gyro_lowpower = false;
	sensor.xl_sens = 4.0 / (1 << 16);		// default 16-bit sensitivity for acceleration
	sensor.gyro_sens = 500.0 / (1 << 16);	// default 16-bit sensitivity for angular velocity
	sensor.i2c_timeout = timeout;		// timeout duration for all sensor I2C transactions

	// Reset device to ensure default values are loaded
	if (IMU_IsConnected(sensor))	// is connected
	{
		IMU_Reset(sensor);
	}
	return sensor;
}

// Check if device is connected by querying Product ID register
bool IMU_IsConnected(IMU sensor)
{
	uint8_t reg = REG_ID;
	uint8_t product_id = 0;
	bool result = false;		// change to "true" if all conditions met

	// only return "true" if all conditions met
	if (HAL_I2C_Master_Transmit(sensor.pointer_i2c_handle,
			IMU_ADDRESS, &reg, 1, sensor.i2c_timeout) == HAL_OK)
	{
		if (HAL_I2C_Master_Receive(sensor.pointer_i2c_handle,
				IMU_ADDRESS, &product_id, 1, sensor.i2c_timeout) == HAL_OK)
		{
			if (product_id == 0x6A)
			{
				result = true;
			}
		}
	}

	// update "connected" field and return result
	sensor.connected = result;
	return result;
}

// Set sampling rate for accelerometer measurements
int XL_SetSamplingRate(IMU sensor, freq rate)
{
	uint8_t reg = REG_CTRL1;
	uint8_t reg_value;
	HAL_StatusTypeDef status;

	// Set target register
	status = HAL_I2C_Master_Transmit(sensor.pointer_i2c_handle,
			IMU_ADDRESS, &reg, 1, sensor.i2c_timeout);
	if (status != HAL_OK) return status;

	// Read register value
	status = HAL_I2C_Master_Receive(sensor.pointer_i2c_handle,
			IMU_ADDRESS, &reg_value, 1, sensor.i2c_timeout);
	if (status != HAL_OK) return status;

	// update upper four bits based on sampling rate
	reg_value = (reg_value & 0x0F) | (rate << 4);

	// Send updated value to register
	uint8_t data[2] = { reg, reg_value };
	status = HAL_I2C_Master_Transmit(sensor.pointer_i2c_handle,
			IMU_ADDRESS, data, 2, sensor.i2c_timeout);

	return status;
}

// Enable low-power mode for accelerometer
int XL_LowPower_Enable(IMU sensor)
{
	return I2C_SetBit(sensor.pointer_i2c_handle,
			IMU_ADDRESS, REG_CTRL6, 4, true, sensor.i2c_timeout);
}

// Disable low-power mode for accelerometer
int XL_LowPower_Disable(IMU sensor)
{
	return I2C_SetBit(sensor.pointer_i2c_handle,
			IMU_ADDRESS, REG_CTRL6, 4, false, sensor.i2c_timeout);
}

/*
* @brief Get accelerometer measurements from IMU
* @param sensor		IMU instance of sensor
* @param timeout 	I2C timeout value in milliseconds
* @retval			Linear acceleration along axis in terms of "g"
*/
float XL_GetX(IMU sensor)
{
	int16_t bitval = ReadTwoBytes(sensor, REG_XL_X, sensor.i2c_timeout);
	return (float) bitval * sensor.xl_sens;  // Convert to units of "g"
}

float XL_GetY(IMU sensor)
{
	int16_t bitval = ReadTwoBytes(sensor, REG_XL_Y, sensor.i2c_timeout);
	return (float) bitval * sensor.xl_sens;  // Convert to units of "g"
}

float XL_GetZ(IMU sensor)
{
	int16_t bitval = ReadTwoBytes(sensor, REG_XL_Z, sensor.i2c_timeout);
	return (float) bitval * sensor.xl_sens;  // Convert to units of "g"
}

// Set sampling rates for gyroscope
int Gyro_SetSamplingRate(IMU sensor, freq rate)
{
	uint8_t reg = REG_CTRL2;
	uint8_t reg_value;
	HAL_StatusTypeDef status;

	// Set target register
	status = HAL_I2C_Master_Transmit(sensor.pointer_i2c_handle,
			IMU_ADDRESS, &reg, 1, sensor.i2c_timeout);
	if (status != HAL_OK) return status;

	// Read register value
	status = HAL_I2C_Master_Receive(sensor.pointer_i2c_handle,
			IMU_ADDRESS, &reg_value, 1, sensor.i2c_timeout);
	if (status != HAL_OK) return status;

	// update upper four bits based on sampling rate
	reg_value = (reg_value & 0x0F) | (rate << 4);

	// Send updated value to register
	uint8_t data[2] = { reg, reg_value };
	status = HAL_I2C_Master_Transmit(sensor.pointer_i2c_handle,
			IMU_ADDRESS, data, 2, sensor.i2c_timeout);

	return status;
}

int Gyro_LowPower_Enable(IMU sensor)
{
	return I2C_SetBit(sensor.pointer_i2c_handle,
				IMU_ADDRESS, REG_CTRL7, 7, true, sensor.i2c_timeout);
}

int Gyro_LowPower_Disable(IMU sensor)
{
	return I2C_SetBit(sensor.pointer_i2c_handle,
				IMU_ADDRESS, REG_CTRL7, 7, false, sensor.i2c_timeout);
}

/*
* @brief Get gyroscope measurements from IMU
* @param sensor		IMU instance of sensor
* @param timeout 	I2C timeout value in milliseconds
* @retval			Angular velocity [degrees per second] about specified axis
*/
float Gyro_GetRoll(IMU sensor)
{
	int16_t bitval = ReadTwoBytes(sensor, REG_GYRO_ROLL, sensor.i2c_timeout);
	return (float) bitval * sensor.gyro_sens;  // Convert to degrees per second
}

float Gyro_GetPitch(IMU sensor)
{
	int16_t bitval = ReadTwoBytes(sensor, REG_GYRO_PITCH, sensor.i2c_timeout);
	return (float) bitval * sensor.gyro_sens;  // Convert to degrees per second
}

float Gyro_GetYaw(IMU sensor)
{
	int16_t bitval = ReadTwoBytes(sensor, REG_GYRO_YAW, sensor.i2c_timeout);
	return (float) bitval * sensor.gyro_sens;  // Convert to degrees per second
}

// Resets physical device by setting "SW_RESET" bit
int IMU_Reset(IMU sensor)
{
	return I2C_SetBit(sensor.pointer_i2c_handle,
			IMU_ADDRESS, REG_CTRL3, 0, true, sensor.i2c_timeout);
}


// Helper function for setting/clearing bits over I2C
int I2C_SetBit(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint8_t reg_addr,
		uint8_t bit_pos, bool enable, uint32_t timeout)
{
    uint8_t reg_val;
    HAL_StatusTypeDef status;

    // Set register address for reading
    status = HAL_I2C_Master_Transmit(hi2c, dev_addr, &reg_addr, 1, timeout);
    if (status != HAL_OK) return status;

    // Read current register value
    status = HAL_I2C_Master_Receive(hi2c, dev_addr, &reg_val, 1, timeout);
    if (status != HAL_OK) return status;

    // Modify bit
    if (enable)
        reg_val |= (1 << bit_pos);   // Set bit
    else
        reg_val &= ~(1 << bit_pos);  // Clear bit

    // Prepare buffer: [register address][new value]
    uint8_t data[2] = { reg_addr, reg_val };
    status = HAL_I2C_Master_Transmit(hi2c, dev_addr, data, 2, timeout);

    return status;
}

// Helper function for reading values from two consecutive registers
int16_t ReadTwoBytes(IMU sensor, uint8_t reg, uint32_t timeout)
{
	uint8_t bytes[2];		// read two bytes
	HAL_StatusTypeDef status;

	// Set target register
	status = HAL_I2C_Master_Transmit(sensor.pointer_i2c_handle, IMU_ADDRESS, &reg, 1, timeout);
	if (status != HAL_OK) return 0;

	// Read values from two consecutive registers
	status = HAL_I2C_Master_Receive(sensor.pointer_i2c_handle, IMU_ADDRESS, bytes, 2, timeout);
	if (status != HAL_OK) return 0;

	// Convert to signed integer -> first byte is lower bits, second byte is upper bits
	return (int16_t) ((bytes[1] << 8) | bytes[0]);
}
