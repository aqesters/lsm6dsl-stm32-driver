# LSM6DSL
STM32 driver for interfacing with the LSM6DSL 6-axis sensor, capable of linear acceleration and gyroscopic measurements.
This library was designed for STM32L4 series microcontrollers using STM32Cube.  

## Important notes
(1) `imu.h` and `imu.c` were designed using STM32L4 I2C HAL drivers in STM32Cube. 
- If STM32Cube is used with a different microcontroller, functionality must be tested with those specific HAL drivers.
- If STM32Cube is not used, a custom I2C driver must be designed and integrated into the source code.

(2) The source code uses the `float` number type. Make sure that floats are supported and enabled in your microcontroller configuration. 

## Usage
When initializing, a pointer to an IMU data type is created (`pSensor` here). The IMU data structure keeps track of the sensor config and status. 
The first argument is the STM32Cube handler for the I2C HAL, `hi2c2` in this case. 
``` c
IMU *pSensor = IMU_Init(&hi2c2, I2C_TIMEOUT);
```

By default, the device is powered down, and low-power modes are disabled. However, we can configure the accelerometer like so:
``` c
XL_SetSamplingRate(pSensor, FREQ_12P5);  // set 12.5 Hz sampling rate, see "imu.h" for other options
XL_LowPower_Enable(pSensor);             // enable low-power mode
```

Similarly, we can configure the gyroscope like this:
``` c
Gyro_SetSamplingRate(pSensor, FREQ_12P5);
Gyro_LowPower_Enable(pSensor);
```

After configuration, measurements are obtained using the following functions. Acceleration outputs are in terms of g-force (e.g., 0.5g). Gyroscope outputs are in degrees per second.
``` c
float ACCEL_X = XL_AccelX(pSensor);            // acceleration along X
float ACCEL_Y = XL_AccelY(pSensor);            // acceleration along Y
float ACCEL_Z = XL_AccelZ(pSensor);            // acceleration along Z
float GYRO_R = Gyro_AngularRateX(pSensor);     // rotational velocity about X-axis
float GYRO_P = Gyro_AngularRateY(pSensor);     // rotational velocity about Y-axis
float GYRO_Y = Gyro_AngularRateZ(pSensor);     // rotational velocity about Z-axis
```

By default, the full-scale range for the accelerometer is +/-2g, and the full-scale range for the gyroscope is +/-250 degrees per second.
This code does not yet support the ability to change the full-scale ranges. 
