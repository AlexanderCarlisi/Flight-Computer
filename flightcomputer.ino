#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BME280.h>

#define SERVO_X_SIGNAL 10
#define SERVO_Y_SIGNAL 2

typedef enum Mode {
  PreInit,        // Default Mode when Code Startsup, Means never got past Startup Function
  OnPad,          // Startup
  PoweredFlight,  // Servoing, Printing
  Coast           // Printing, Deploy shoot, blah blah blah
} Mode;

Mode flightMode;

Adafruit_MPU6050 mpu;
Adafruit_BME280 bme;
Servo ServoX;
Servo ServoY;

float pitch = 0.0, roll = 0.0, yaw = 0.0;

float setpointPitch = -90, setpointRoll = 0; // Patty you should update the setpoint to fix the tilting.
float Kp = 2., Ki = 0.05, Kd = 0.1;
float integralPitch = 0, integralRoll = 0;
float prevErrorPitch = 0, prevErrorRoll = 0;

// Persistent values for PID
//float integral = 0;
//float previousError = 0;

unsigned long lastTime = 0;


float PID(float setangle, float input, float dt, float &integral, float &previousError){
  float error = setangle - input;
  integral += error * dt;
  float derivative = (error - previousError) / dt;
  float output = Kp * error + Ki * integral + Kd * derivative;
  previousError = error;
  return output;
}

void checkSerialForModeChange() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Read full line until Enter
    input.trim(); // Remove spaces/newlines

    if (input.length() > 0 && input.charAt(0) >= '0' && input.charAt(0) <= '3') {
      int modeNumber = input.toInt();
      if (modeNumber >= 0 && modeNumber <= 3) {
        if (modeNumber != flightMode) {  // Only update if mode is actually different
          flightMode = (Mode)modeNumber;
          Serial.print("Flight mode set to: ");
          switch (flightMode) {
            case PreInit:        Serial.println("PreInit"); break;
            case OnPad:          Serial.println("OnPad"); break;
            case PoweredFlight:  Serial.println("PoweredFlight"); break;
            case Coast:          Serial.println("Coast"); break;
          }
        }
      } else {
        Serial.println("Invalid mode. Use: 0=PreInit, 1=OnPad, 2=PoweredFlight, 3=Coast");
      }
    }
  }

}

void setup() {

  // If servos are still slow, try raising the Bitrate.
  do {
    Serial.begin(38400); // 9600 - 115200
  } while (!Serial);

  flightMode = PreInit;

  int out = -1;
  do {
    out = mpu.begin();
    switch(out) {
      case MPU6050_BEGIN_ALL_GOOD: {
        Serial.print("\n>>> Successfully Initialized MPU6050 <<<\n");
        break;
      }
      case MPU6050_BEGIN_INVALID_CHIP: {
        Serial.print("\n>>> Error Initializing MPU6050. Detected Chip is not an MPU6050 <<<\n");
        break;
      }
      case MPU6050_BEGIN_NOT_FOUND: {
        Serial.print("\n>>> Error Instantiating MPU6050. No I2C Connection detected on provided port <<<\n");
        break;
      }
      default: {
        Serial.print("\n>>> MPU6050, Impossible output. <<<\n");
        break;
      }
    }
    delay(100);
  } while (out != MPU6050_BEGIN_ALL_GOOD);

  out = -1;
  do {
    out = bme.begin(0x76);
    switch(out) {
      case BME280_BEGIN_ALL_GOOD: {
        Serial.print("\n>>> Successfully Initialized BME280 <<<\n");
        break;
      }
      case BME280_INIT_INCORRECT_CHIP_ID: {
        Serial.print("\n>>> Error Initializing BME280. Detected Chip is not a BME280 <<<\n");
        break;
      }
      case BME280_BEGIN_I2C_NOT_DETECTED: {
        Serial.print("\n>>> Error Instantiating BME280. No I2C Connection detected on provided port <<<\n");
        break;
      }
      case BME280_BEGIN_SPI: {
        Serial.print("\n>>> BME280 Running on SPI and not I2C <<<\n");
        break;
      }
      default: {
        Serial.print("\n>>> BME280, Impossible output. <<<\n");
        break;
      }
    }
    delay(100);
  } while (out != BME280_BEGIN_ALL_GOOD);

  // Servo Signal Pins
  ServoX.attach(SERVO_X_SIGNAL);
  ServoY.attach(SERVO_Y_SIGNAL);

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("Sensors initialized.");

  flightMode = OnPad;

  delay(100);
  lastTime = millis();
}


void serialData(float altitude, float temperature, float pressure, float servoAngleX, float servoAngleY, float gx, float gy, float gz) {
  // Each char is one Byte. This could be causing the slowdown.
  // Output
  Serial.print(", Pitch: "); Serial.print(pitch);
  Serial.print("°, Roll: "); Serial.print(roll);
  Serial.print("°, Yaw: "); Serial.print(yaw);
  //Serial.print("°, Altitude: "); Serial.print(altitude);
  //Serial.print(" m, Temp: "); Serial.print(temperature);
  //Serial.print(" °C, Pressure: "); Serial.print(pressure);
  //Serial.print(" hPa");
  //Serial.print("°, Servo AngleX: "); Serial.print(servoAngleX);
  //Serial.print("°, Servo AngleY: "); Serial.print(servoAngleY);
  
  Serial.print("°, Gyro AngleX:  "); Serial.print(gx);
  Serial.print("°, Gyro AngleY:  "); Serial.print(gy);
  Serial.print("°, Gyro Anglez:  "); Serial.print(gz);
  
  Serial.println();
}


void loop() {
  Serial.print("worcking");
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);   // This could be causing Slowdown, can't see the code so idk.
  
  // All this is just math, it wont cause the slowdown.
  // Calculate pitch angle from accelerometer
  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;

  float gx = gyro.gyro.x * 180.0 / PI;
  float gy = gyro.gyro.y * 180.0 / PI;
  float gz = gyro.gyro.z * 180.0 / PI;

  float dt = (millis() - lastTime) / 1000.0;

  float accPitch = atan2(-ax, az) * 180.0 / PI;
  float accRoll  = atan2(ay, az) * 180.0 / PI;

  float alpha = 0.98;
  pitch = alpha * (pitch + gy * dt) + (1 - alpha) * accPitch;
  roll  = alpha * (roll + gx * dt) + (1 - alpha) * accRoll;
  yaw += gz * dt;// relative yaw

  float temperature = bme.readTemperature();
  float pressure = bme.readPressure() / 100.0F; // hPa
  float altitude = bme.readAltitude(1013.25);   // Sea-level pressure (hPa)

  float outputX = PID(setpointPitch, pitch, dt, integralPitch, prevErrorPitch);
  float outputY = PID(setpointRoll , roll, dt, integralRoll, prevErrorRoll);

  int servoAngleX = map(outputX, -90, 90, 0, 180);
  int servoAngleY = map(outputY, -90, 90, 0, 180);

  servoAngleX = constrain(servoAngleX, 0, 180);
  servoAngleY = constrain(servoAngleY, 0, 180);
  
  //mode switch logic
  checkSerialForModeChange();
  Serial.print("worcking");
  switch(flightMode) {
    case OnPad: {
      Serial.print("Mode: OnPad");
      break;
    }
    case PoweredFlight: {
      ServoX.write(servoAngleY);
      ServoY.write(servoAngleX);
      Serial.print("Mode: PoweredFlight");
      serialData(altitude, temperature, pressure, servoAngleX, servoAngleY, gx, gy, gz);
      break;
    }
    case Coast: {
      Serial.print("Mode: Coast");
      serialData(altitude, temperature, pressure, servoAngleX, servoAngleY, gx, gy, gz);
      break;
    }
    default: {
      Serial.print("Mode: "); Serial.print(flightMode);
      break;
    }
  }

  lastTime = millis();
  delay(10); // Look into maybe removing the 10ms delay to remove the slowdown.
}