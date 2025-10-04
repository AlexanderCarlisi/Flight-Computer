/// TODO:
/// Simplify Bitwise operations, went crazy debugging, now clean the mess up.

#include "mpu6050.hpp"
#include <Wire.h>
#include <Arduino.h>


void mpu6050_init() {
  // Wake up the MPU6050 (clear sleep bit)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_REG_PWR_MGMT_1);
  Wire.write(0);
  Wire.endTransmission(true);

  // // write to accel config
  // Wire.beginTransmission(MPU6050_ADDR);
  // Wire.write(MPU6050_REG_ACCEL_CONFIG);
  // Wire.endTransmission(false);
  // Wire.requestFrom(MPU6050_ADDR, 1);
  // uint8_t reg = (Wire.read() & 0b11111000) | 0b00011000; 
  // Wire.beginTransmission(MPU6050_ADDR);
  // Wire.write(MPU6050_REG_ACCEL_CONFIG);
  // Wire.write(reg);
  // Wire.endTransmission(true);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_REG_ACCEL_CONFIG);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 1);
  uint8_t reg = Wire.read();
  // _bits = 2, _shift = 3
  uint8_t mask = (1 << (2)) - 1;
  uint8_t data = 0b11; // 3
  mask <<= 3;
  reg &= ~mask;
  reg |= data << 3;
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_REG_ACCEL_CONFIG);
  Wire.write(reg);
  Wire.endTransmission(true);
  
  // Read back Accelerometer config
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_REG_ACCEL_CONFIG);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 1, true);
  uint8_t accel_readback = Wire.read();
  Serial.print("Accel Config: 0b");
  Serial.println(accel_readback, BIN);

  delay(5000);
}

MPU6050Data mpu6050_get() {
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

  data.gx = raw_gx * GYRO_SCALE;  // °/s
  data.gy = raw_gy * GYRO_SCALE;
  data.gz = raw_gz * GYRO_SCALE;

  return data;
}
