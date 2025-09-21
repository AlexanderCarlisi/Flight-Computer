#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

#define MPU6050_ADDR 0x68
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define ACCEL_SCALE  (9.81 / 16384.0)   // m/s^2 per LSB
#define GYRO_SCALE   (1.0 / 131.0)      // °/s per LSB

/// @struct MPU6050Data
/// Gyro Values - Degrees
/// Accel Values - m/s/s
typedef struct MPU6050Data {
  int16_t gx, gy, gz, ax, ay, az;
} MPU6050Data;

void mpu6050_init();
MPU6050Data mpu6050_get();

#endif // MPU6050_H
