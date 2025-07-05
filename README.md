# LSM6DSL
STM32 driver for interfacing with the LSM6DSL 6-axis sensor, capable of linear acceleration and gyroscopic measurements.
This library was designed for STM32L4 series microcontrollers using STM32CubeIDE.  

## Important notes
(1) `imu.h` and `imu.c` were designed using STM32L4 I2C HAL drivers in STM32CubeIDE. 
- If STM32CubeIDE is used with a different microcontroller, functionality must be tested with those specific HAL drivers.
- If STM32CubeIDE is not used, a custom I2C driver must be designed and integrated into the source code.

(2) There is a `.zip` file in this repo that contains a working copy of the STM32CubeIDE workspace with the source code.
If using STM32CubeIDE, copying this workspace into the `STM32CubeIDE/` directory should allow the code to be built and compiled.

(3) The source code uses the `float` type. Make sure that floats are supported and enabled by your configuration. 

## Usage
Initialize the IMU data structure ("hi2c2" is the handler for the I2C HAL). By default, the device is powered down and low-power modes are disabled.
``` c
IMU sensor = IMU_Init(&hi2c2, I2C_TIMEOUT);
```

To start accelerometer measurements, the sampling rate must be configured. This determines the frequency at which the output register is updated. The sampling rate options are found in `imu.h`.
``` c
XL_SetSamplingRate(sensor, FREQ_12P5);  // set 12.5 Hz sampling rate
XL_LowPower_Enable(sensor);             // enable low-power mode
```

There are analogous functions for the gyroscope.
``` c
Gyro_SetSamplingRate(sensor, FREQ_12P5);
Gyro_LowPower_Enable(sensor);
```

After configuration, measurements are obtained using the following functions. Acceleration outputs are in terms of g-force (e.g., 0.5g). Gyroscope outputs are in degrees per second.
``` c
float ACCEL_X = XL_GetX(sensor);        // acceleration along X
float ACCEL_Y = XL_GetY(sensor);        // acceleration along Y
float ACCEL_Z = XL_GetZ(sensor);        // acceleration along Z
float GYRO_R = Gyro_GetRoll(sensor);    // rotational velocity about X-axis
float GYRO_P = Gyro_GetPitch(sensor);   // rotational velocity about Y-axis
float GYRO_Y = Gyro_GetYaw(sensor);     // rotational velocity about Z-axis
```

By default, the full-scale range for the accelerometer is +/-2g, and the full-scale range for the gyroscope is +/-250 degrees per second.
These ranges can be changed on the device, but the source code does not have this function yet. 
