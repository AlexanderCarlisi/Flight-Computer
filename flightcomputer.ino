#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BME280.h>

#define SERVO_X_PIN 10
#define SERVO_Y_PIN 2
#define EEPROM_ADDR 0x50
#define BME_ADDR    0x76

// Sensors
Adafruit_MPU6050 mpu;
Adafruit_BME280 bme;
Servo ServoX;
Servo ServoY;

// Calc dt in loop
unsigned long lastTime = 0;


///
/// PID Funcs and Vars
///

// PID Constants
const float setpointPitch = -90, setpointRoll = 0;
const float Kp = 2., Ki = 0.05, Kd = 0.1;

// PID Variables
float integralPitch = 0, integralRoll = 0;
float prevErrorPitch = 0, prevErrorRoll = 0;

float PID(float setangle, float input, float dt, float &integral, float &previousError){
  float error = setangle - input;
  integral += error * dt;
  float derivative = (error - previousError) / dt;
  float output = Kp * error + Ki * integral + Kd * derivative;
  previousError = error;
  return output;
}


///
/// Flight State Machine
///

#define MODE_CHANGE         1     // 0 = Velocity based, 1 = Serial based
#define FREEFALL_THRESHOLD  2.0   // m/s/s, checks accelerometer  TODO: tune
#define DESCENT_THRESHOLD   0.5   // Altitude velocity threshold to be considered at Apogee.
#define APOGEE_COUNT        5     // Consecutive ticks at DESCENT_THREASHOLD, till considered at Apogee
#define _G                  9.81  // Earth, Gravity TS

typedef enum Mode {
  PreInit,        // Initializing Arduino code
  OnPad,          // OnPad, awaiting Launch
  PoweredFlight,  // Launched: Servoing
  Coast           // Deployed Parachute, coasting down.
} Mode;
Mode flightMode;

void deploy_parachute() {
  // TODO:
}

void abort() {
  // TODO:
  deploy_parachute();
}

void mode_change_serial() {
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

bool parachute_deployed = false;
float previousAltitude;
int descent_count;

void mode_change_velocity(float altitude, float dt, float ax, float ay, float az) {
  // Check for Apogee for Parachute Deployment
  if (!parachute_deployed && flightMode == PoweredFlight) {

    // Check barometer
    float altitudeVelocity = (altitude - previousAltitude) / dt;
    if (altitudeVelocity <= DESCENT_THRESHOLD) descent_count++;
    else descent_count = 0;
    bool baroCheck = descent_count >= DESCENT_THRESHOLD;

    // Check accelerometer
    float totalAcc = sqrt(ax*ax + ay*ay + az*az); // Net forces
    bool accelCheck = totalAcc <= FREEFALL_THRESHOLD;

    if (baroCheck && accelCheck) {
      deploy_parachute();
      parachute_deployed = true;
      flightMode = Coast;
    }
  }
}



///
/// Logging Framework
///

typedef struct LoggedState {
  float dt, ax, ay, az,
        gx, gy, gz,
        temperature, pressure, altitude,
        accPitch, accRoll,
        pitch, roll, yaw,
        pidOutX, pidOutY,
        servoAngleX, servoAngleY;
  int descent_count;
  bool parachute_deployed;
  Mode mode;
} LoggedState;



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
    out = bme.begin(BME_ADDR);
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
  ServoX.attach(SERVO_X_PIN);
  ServoY.attach(SERVO_Y_PIN);

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("Sensors initialized.");

  flightMode = OnPad;
  lastTime = millis();
}


// Globals for Loop
float yaw = 0;

void loop() {
  // Update Sensor Values
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  
  float dt = (millis() - lastTime) / 1000.0;

  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;

  float gx = gyro.gyro.x * 180.0 / PI;
  float gy = gyro.gyro.y * 180.0 / PI;
  float gz = gyro.gyro.z * 180.0 / PI;

  float temperature = bme.readTemperature();
  float pressure = bme.readPressure() / 100.0F; // hPa
  float altitude = bme.readAltitude(1013.25);   // Sea-level pressure (hPa)

  float accPitch = atan2(-ax, az) * 180.0 / PI;
  float accRoll  = atan2(ay, az) * 180.0 / PI;
  
  // Complementary filter contant between the Gyro and Accelerometer
  float alpha = 0.98; // TODO: may need tuning

  float pitch = alpha * (pitch + gy * dt) + (1 - alpha) * accPitch;
  float roll = alpha * (roll + gx * dt) + (1 - alpha) * accRoll;
  yaw += gz * dt;
  
  // Servo output
  float outputX = PID(setpointPitch, pitch, dt, integralPitch, prevErrorPitch);
  float outputY = PID(setpointRoll , roll, dt, integralRoll, prevErrorRoll);

  int servoAngleX = map(outputX, -90, 90, 0, 180);
  int servoAngleY = map(outputY, -90, 90, 0, 180);

  servoAngleX = constrain(servoAngleX, 0, 180);
  servoAngleY = constrain(servoAngleY, 0, 180);
  
  // mode switch logic
  if (MODE_CHANGE == 0) {
    mode_change_serial();
  } else {
    mode_change_velocity(altitude, dt, ax, ay, az);
  }
  
  // Perform actions based on mode
  switch(flightMode) {
    case OnPad: {
      Serial.print("Mode: OnPad");
      break;
    }
    case PoweredFlight: {
      ServoX.write(servoAngleY);
      ServoY.write(servoAngleX);
      break;
    }
    case Coast: {
      break;
    }
    default: {
      break;
    }
  }

  lastTime = millis();

  // Logging
  LoggedState ls = {
    dt, ax, ay, az, gx, gy, gz, temperature, pressure, altitude,
    accPitch, accRoll, pitch, roll, yaw, outputX, outputY, servoAngleX, 
    servoAngleY, descent_count, parachute_deployed, flightMode
  };
}


void serialData() {
  
}

