/// TODO: Cleanup definitions, stick to Hex
/// ~maybe add temperature here
/// cleanup function names, get? get what?
/// Datasheet: https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/RM-MPU-6000A.pdf
#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

/// Addresses
#define MPU6050_ADDR 0x68
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_GYRO_CONFIG 0x1B

// Pulled from Adafruit Repo
#define ACCEL_SCALE  1/2048     // m/s^2 per LSB
#define GYRO_SCALE   1/16.4     // °/s per LSB

/// Config Register Masks
#define MPU6050_SEL_16G 0x18
#define MPU6050_SEL_8G  0x10
#define MPU6050_SEL_4G  0x08
#define MPU6050_SEL_2G  0x00
#define MPU6050_X_ST    0x80
#define MPU6050_Y_ST    0x40
#define MPU6050_Z_ST    0x20

/// REG PWR MGMT 1 Register Masks
#define MPU6050_DEVICE_RESET  0x80
#define MPU6050_SLEEP         0x40
#define MPU6050_TEMP_DISABLE  0x08

/// Beginning of Accel Measurements
/// Accel, Temperature, Gyro
/// XX,YY,ZZ,TT,XX,YY,ZZ
/// From 0x38 - 0x48
/// 14 Bytes
#define MPU6050_REG_ACCEL_XOUT_H 0x3B


/// @struct MPU6050Data
/// Gyro Values - Degrees
/// Accel Values - m/s/s
typedef struct MPU6050Data {
  int16_t gx, gy, gz, ax, ay, az;
} MPU6050Data;

void mpu6050_init();
MPU6050Data mpu6050_read();

#endif // MPU6050_H
