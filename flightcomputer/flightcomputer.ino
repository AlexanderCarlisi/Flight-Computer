#include <Wire.h>
#include <Servo.h>
#include <Adafruit_BME280.h>
#include <RF24.h>
#include "log.hpp"
#include "mpu6050.hpp"

///
/// Servos and Sensors
///

#define SERVO_X_PIN   10
#define SERVO_Y_PIN   2
#define EEPROM_ADDR   0x50
#define BME_ADDR      0x76

Adafruit_BME280 bme;
Servo ServoX;
Servo ServoY;

void bme_init();


///
/// PID
///
const float setpointPitch = -90, setpointRoll = 0;
const float Kp = 2., Ki = 0.05, Kd = 0.1;
float integralPitch = 0, integralRoll = 0;
float prevErrorPitch = 0, prevErrorRoll = 0;

float PID(float setangle, float input, float dt, float &integral, float &previousError);


///
/// Flight State Machine
///

#define MODE_CHANGE         1     // 0 = Velocity based, 1 = Serial based
#define FREEFALL_THRESHOLD  2.0   // m/s/s, checks accelerometer  TODO: tune
#define DESCENT_THRESHOLD   0.5   // Altitude velocity threshold to be considered at Apogee.
#define APOGEE_COUNT        5     // Consecutive ticks at DESCENT_THREASHOLD, till considered at Apogee
#define ABORT_DEGREES       45    // TODO: Tune, how many degrees off on Gyro until we Abort.
#define _G                  9.81  // Earth, Gravity TS

FlightState state;
float previous_altitude;

void deploy_parachute();
void abort();
void mode_change_serial();
void move_change_velocity();


///
/// RADIO 
///
#define PIPE      0
#define TX_ADDR   "PAT01"
#define RX_ADDR   "PAT02"
#define RADIO_CE  0
#define RADIO_CSN 0

// RF24 radio(RADIO_CE, RADIO_CSN);
void radio_init();


void setup() {
  // If servos are still slow, try raising the Bitrate.
  do {
    Serial.begin(38400); // 9600 - 115200
  } while (!Serial);

  state.mode = PreInit;
  
  Wire.begin();
  mpu6050_init();
  bme_init();
  radio_init();
  ServoX.attach(SERVO_X_PIN);
  ServoY.attach(SERVO_Y_PIN);

  serialPrint("\n>>> Sensors initialized <<<\n");
  state.mode = OnPad;
}

// Loop globals
unsigned long last_time = 0;

void loop() {
  float currentTime = millis();
  state.dt = (currentTime - last_time) / 1000.0;
  
  // Update Sensor Values
  MPU6050Data mpu = mpu6050_read();

  state.ax = mpu.ax;
  state.ay = mpu.ay;
  state.az = mpu.az;

  state.gx = mpu.gx * 180.0 / PI;
  state.gy = mpu.gy * 180.0 / PI;
  state.gz = mpu.gz * 180.0 / PI;

  state.temperature = bme.readTemperature();
  state.pressure = bme.readPressure() / 100.0F; // hPa
  state.altitude = bme.readAltitude(1013.25);   // Sea-level pressure (hPa)

  state.accPitch = atan2(-state.ax, state.az) * 180.0 / PI;
  state.accRoll  = atan2(state.ay, state.az) * 180.0 / PI;
  
  // Complementary filter contant between the Gyro and Accelerometer
  float alpha = 0.98; // TODO: may need tuning

  state.pitch = alpha * (state.pitch + state.gy * state.dt) + (1 - alpha) * state.accPitch;
  state.roll = alpha * (state.roll + state.gx * state.dt) + (1 - alpha) * state.accRoll;
  state.yaw += state.gz * state.dt;
  
  // Servo output
  state.pidOutX = PID(setpointPitch, state.pitch, state.dt, integralPitch, prevErrorPitch);
  state.pidOutY = PID(setpointRoll , state.roll, state.dt, integralRoll, prevErrorRoll);

  state.servoAngleX = map(state.pidOutX, -90, 90, 0, 180);
  state.servoAngleY = map(state.pidOutY, -90, 90, 0, 180);

  state.servoAngleX = constrain(state.servoAngleX, 0, 180);
  state.servoAngleY = constrain(state.servoAngleY, 0, 180);
  
  // mode switch logic
  if (MODE_CHANGE == 0) {
    mode_change_serial();
  } else {
    mode_change_velocity();
  }
  
  // Perform actions based on mode
  switch(state.mode) {
    case OnPad: {
      break;
    }
    case PoweredFlight: {
      ServoX.write(state.servoAngleX);
      ServoY.write(state.servoAngleY);
      break;
    }
    case Coast: {
      break;
    }
    default: {
      break;
    }
  }

  // Logging
  if (currentTime - getPrevLogTime() >= LOGGING_PERIOD_MS) {
    logStateToSD(state);
    updatePrevLogTime();
  }

  last_time = millis();
}

void bme_init() {
  int out = -1;
  do {
    out = bme.begin(BME_ADDR);
    switch(out) {
      case BME280_BEGIN_ALL_GOOD: {
        serialPrint("\n>>> Successfully Initialized BME280 <<<\n");
        break;
      }
      case BME280_INIT_INCORRECT_CHIP_ID: {
        serialPrint("\n>>> Error Initializing BME280. Detected Chip is not a BME280 <<<\n", true);
        break;
      }
      case BME280_BEGIN_I2C_NOT_DETECTED: {
        serialPrint("\n>>> Error Instantiating BME280. No I2C Connection detected on provided port <<<\n", true);
        break;
      }
      case BME280_BEGIN_SPI: {
        serialPrint("\n>>> BME280 Running on SPI and not I2C <<<\n", true);
        break;
      }
      default: {
        serialPrint("\n>>> BME280, Impossible output. <<<\n", true);
        break;
      }
    }
    delay(100);
  } while (out != BME280_BEGIN_ALL_GOOD);
}

void radio_init() {
  // // Radio setup
  // out = -1;
  // do {
  //   out = radio.begin();
  //   switch(out) {
  //     case RF24_BEGIN_SUCCESS: {
  //       serialPrint("\n>>> RF24 Successfully Initialized <<<\n");
  //       break;
  //     }
  //     case RF24_BEGIN_ERROR_CE_INVALID_PIN: {
  //       serialPrint("\n>>> RF24 INVALID CE PIN <<<\n", true);
  //       break;
  //     }
  //     case RF24_BEGIN_ERROR_CSN_INVALID_PIN: {
  //       serialPrint("\n>>> RF24 INVALID CSN PID <<<\n", true);
  //       break;
  //     }
  //     case RF24_BEGIN_ERROR_INIT_RADIO_BAD_CONFIG: {
  //       serialPrint("\n>>> RF24 BAD CONFIG <<<\n", true);
  //       break;
  //     }
  //     default: {
  //       serialPrint("\n>>> RF24 Impossible output <<<\n", true);
  //       break;
  //     }
  //     // Delay is embedded withing radio::_init_radio
  //   } 
  // } while(out != RF24_BEGIN_SUCCESS);
}

float PID(float setangle, float input, float dt, float &integral, float &previousError){
  float error = setangle - input;
  integral += error * dt;
  float derivative = (error - previousError) / dt;
  float output = Kp * error + Ki * integral + Kd * derivative;
  previousError = error;
  return output;
}

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
        if (modeNumber != state.mode) {  // Only update if mode is actually different
          state.mode = (Mode)modeNumber;
          Serial.print("Flight mode set to: ");
          switch (state.mode) {
            case PreInit:        Serial.println("PreInit"); break;
            case OnPad:          Serial.println("OnPad"); break;
            case PoweredFlight:  Serial.println("PoweredFlight"); break;
            case Coast:          Serial.println("Coast"); break;
          }
        }
      } else {
        Serial.println("Invalid mode. Use: 0=PreInit, 1=OnPad, 2=PoweredFlight, 3=Coast");
        serialPrint("\n>>> INVALID INPUT FROM STATION <<<\n", true);
      }
    }
  }
}

void mode_change_velocity() {
  // Check for Apogee for Parachute Deployment | RECOVERY
  if (!state.parachute_deployed && state.mode == PoweredFlight) {
    // Check barometer
    float altitudeVelocity = (state.altitude - previous_altitude) / state.dt;
    if (altitudeVelocity <= DESCENT_THRESHOLD) state.descent_count++;
    else state.descent_count = 0;
    bool baroCheck = state.descent_count >= DESCENT_THRESHOLD;

    // Check accelerometer
    // Net force calculation
    float totalAcc = sqrt(state.ax*state.ax + state.ay*state.ay + state.az*state.az);
    bool accelCheck = totalAcc <= FREEFALL_THRESHOLD;

    if (baroCheck && accelCheck) {
      deploy_parachute();
      state.parachute_deployed = true;
      state.mode = Coast;
    }
  }

  // Check conditions for Abort Command
  // TODO: Tune these conditions
  if (!state.abort && (abs(state.yaw) >= ABORT_DEGREES || abs(state.pitch) >= ABORT_DEGREES)) {
    abort();
    state.abort = true;
  }
}
