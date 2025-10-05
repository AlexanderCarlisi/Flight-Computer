#include "mpu6050.hpp"
#include <Wire.h>
#include <Arduino.h>

bool mpu6050_reset_finished() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_REG_PWR_MGMT_1);
  Wire.endTransmission(true);
  Wire.requestFrom(MPU6050_ADDR, 1, true);
  byte reg = Wire.read();
  return reg == 0;
}

void mpu6050_reset() {
  do {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_REG_PWR_MGMT_1);
    Wire.write(MPU6050_DEVICE_RESET);
    Wire.endTransmission(true);
    delay(500);
  } while(!mpu6050_reset_finished());
}

void mpu6050_init() {
  mpu6050_reset();
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_ACCEL_CONFIG);
  Wire.write(MPU6050_SEL_16G); // Normal
  // Wire.write(MPU6050_SEL_16G | MPU6050_X_ST | MPU6050_Y_ST | MPU6050_Z_ST); // Test
  Wire.endTransmission(true);
}

MPU6050Data mpu6050_read() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);

  MPU6050Data data;

  int16_t raw_ax = (Wire.read() << 8) | Wire.read();
  int16_t raw_ay = (Wire.read() << 8) | Wire.read();
  int16_t raw_az = (Wire.read() << 8) | Wire.read();

  // Skip Temp
  Wire.read();
  Wire.read();

  int16_t raw_gx = (Wire.read() << 8) | Wire.read();
  int16_t raw_gy = (Wire.read() << 8) | Wire.read();
  int16_t raw_gz = (Wire.read() << 8) | Wire.read();

  // Scale to SI units
  data.ax = raw_ax * ACCEL_SCALE;
  data.ay = raw_ay * ACCEL_SCALE;
  data.az = raw_az * ACCEL_SCALE;

  data.gx = raw_gx * GYRO_SCALE;
  data.gy = raw_gy * GYRO_SCALE;
  data.gz = raw_gz * GYRO_SCALE;

  return data;
}
