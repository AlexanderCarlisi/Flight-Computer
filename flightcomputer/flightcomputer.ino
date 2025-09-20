#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BME280.h>
#include <SD.h>
#include <RF24.h>
#include <nRF24L01.h>

#define SERVO_X_PIN   10
#define SERVO_Y_PIN   2
#define EEPROM_ADDR   0x50
#define BME_ADDR      0x76

// Sensors
Adafruit_MPU6050 mpu;
Adafruit_BME280 bme;
Servo ServoX;
Servo ServoY;



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
#define ABORT_DEGREES       45    // TODO: Tune, how many degrees off on Gyro until we Abort.
#define _G                  9.81  // Earth, Gravity TS

typedef enum Mode {
  PreInit,        // Initializing Arduino code
  OnPad,          // OnPad, awaiting Launch
  PoweredFlight,  // Launched: Servoing
  Coast           // Deployed Parachute, coasting down.
} Mode;

/// @struct FlightState
/// size on Uno: 81 bytes
typedef struct FlightState {
  float dt, ax, ay, az,
        gx, gy, gz,
        temperature, pressure, altitude,
        accPitch, accRoll,
        pitch, roll, yaw,
        pidOutX, pidOutY,
        servoAngleX, servoAngleY;
  int descent_count;
  bool parachute_deployed;
  bool abort;
  Mode mode;
} FlightState;
FlightState state;

// Helper globals
float previous_altitude;

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



///
/// Logging Framework
///
#define PRINT_TO_SERIAL   false
#define SD_CS_PIN         0
#define SD_LOG_PATH       "/logs"
#define SD_LOG_FILENAME   "log"
#define SD_LOG_EXT        ".pat"
#define LOG_START_BYTE    '$'     // Signifies the call of Setup on Arduino.
#define LOG_SETUP_SUCCESS '%'     // Setup Passed.
#define LOGGING_PERIOD_MS 1000
unsigned long previous_log_time = 0;
String log_filename = "";

// Current position in ROM
// unsigned int eeprom_write_addr = 0; // Start logging @0x00
// unsigned long logging_period_ms = 1000; // Minimum 5ms for write time
// size_t eeprom_capacity_bytes = 2000;

// void eepromWrite() {
//   // Serialize data into bytes
//   byte* ptr = (byte*) &state;
//   for (unsigned int i = 0; i < sizeof(FlightState); i++) {
//     Wire.beginTransmission(EEPROM_ADDR);
//     Wire.write((int)(eeprom_write_addr >> 8)); // high byte
//     Wire.write((int)(eeprom_write_addr & 0xFF));
//     Wire.write(ptr[i]);
//     Wire.endTransmission();
//     eeprom_write_addr++;
//     // dont need to delay here, logging period should encapsulate this
//   }
// }

// TODO:
// void eepromRead() -> buffer*, size_t length
// return the whole eeprom buffer or not return but like set ptr to it

// void eepromToSSD() {
//   // Check for Logs folder
//   if (!SD.exists(SD_LOG_PATH))
//     SD.mkdir(SD_LOG_PATH);
//
//   File logsDir = SD.open(SD_LOG_PATH);
//   if (!logsDir.isDirectory()) {
//     SD.remove(SD_LOG_PATH);
//     SD.mkdir(SD_LOG_PATH);
//     logsDir = SD.open(SD_LOG_PATH);
//   }
//
//   // Find log files, and accordingly name new one
//   int count = 0;
//   while(true) {
//     File entry = logsDir.openNextFile();
//     if (!entry) break;
//     else count++;
//   }
//
//   File logFile;
//   while(true) {
//     logFile = SD.open(
//       strcat(strcat(strcat(SD_LOG_PATH, SD_LOG_FILENAME), count), SD_LOG_EXT),
//       FILE_WRITE);
//     if (logFile) break;
//     // TODO: Better error logging here :O
//     else Serial.println("\n>>> Error creating Log File <<<\n");
//   }
//
//   // Write EEPROM to SD Card
//   // GET EEPROM buffer and length of buffer.
//   // logFile.write(/**buff, len*/);
//   logFile.close();
//   logsDir.close();
// }
//


bool logCreateFile() {
  // Find log files, and accordingly name new one
  logsDir = SD.open(SD_LOG_PATH);
  int count = 0;
  while(true) {
    File entry = logsDir.openNextFile();
    if (!entry) break;
    else count++;
  }
  
  // Write to log file, or return error
  log_filename = SD_LOG_PATH + SD_LOG_FILENAME + String(count) + SD_LOG_EXT;
  File logFile = SD.open(log_filename, FILE_WRITE);
  if(logFile) {
    logFile.write(LOG_START_BYTE);
    logFile.close();
    return true;
  }
  else {
    Serial.println("\n>>> Error creating Log File <<<\n");
    logFile.close();
    return false;
  }
}


bool logInit() {
  // Check for Logs folder
  if (!SD.exists(SD_LOG_PATH)) SD.mkdir(SD_LOG_PATH);

  File logsDir = SD.open(SD_LOG_PATH);
  if (!logsDir) {
    Serial.println("\n>>> Error creating Log Directory <<<\n");
    return false;
  }
  // If that thang aint a folder, fix it
  if (!logsDir.isDirectory()) {
    SD.remove(SD_LOG_PATH);
    SD.mkdir(SD_LOG_PATH);
    logsDir = SD.open(SD_LOG_PATH);
  }
  logsDir.close();
  return logCreateFile();
}


void logStateToSD() {
  File logFile = SD.open(log_filename);
  logFile.write((uint8_t*)&state, sizeof(state));
  logFile.close();
}


void logCharToSD(char c) {
  File logFile = SD.open(log_filename);
  logFile.write(c);
  logFile.close();
}


void logErrToSD(String err) {
  File logFile = SD.poen(log_filename);
  logFile.print(err);
  logFile.close();
}


void serialPrint(String message, bool err = false) {
  if (PRINT_TO_SERIAL || err) Serial.print(message);
  if (err) logErrToSD(message);
}


///
/// RADIO 
///
#define PIPE      0
#define TX_ADDR   "PAT01"
#define RX_ADDR   "PAT02"
#define RADIO_CE  0
#define RADIO_CSN 0

RF24 radio(RADIO_CE, RADIO_CSN);


void setup() {

  // If servos are still slow, try raising the Bitrate.
  do {
    Serial.begin(38400); // 9600 - 115200
  } while (!Serial);

  state.mode = PreInit;

  int out = -1;
  do {
    out = int(SD.begin(SD_CS_PIN));
    if (out == 1) {
      serialPrint("\n>>> Successfully Initialized SD Card <<<\n");
      if (!logInit()) {
        continue;
      }
    } else {
      serialPrint("\n>>> Failed to Initalize SD Card <<<\n", true);
      delay(3000);
    }

  } while(out != 1)

  do {
    out = mpu.begin();
    switch(out) {
      case MPU6050_BEGIN_ALL_GOOD: {
        serialPrint("\n>>> Successfully Initialized MPU6050 <<<\n");
        break;
      }
      case MPU6050_BEGIN_INVALID_CHIP: {
        serialPrint("\n>>> Error Initializing MPU6050. Detected Chip is not an MPU6050 <<<\n", true);
        break;
      }
      case MPU6050_BEGIN_NOT_FOUND: {
        serialPrint("\n>>> Error Instantiating MPU6050. No I2C Connection detected on provided port <<<\n", true);
        break;
      }
      default: {
        serialPrint("\n>>> MPU6050, Impossible output. <<<\n", true);
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

  // Servo Signal Pins
  ServoX.attach(SERVO_X_PIN);
  ServoY.attach(SERVO_Y_PIN);
  
  // MPU setup
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  serialPrint("\n>>> Sensors initialized <<<\n");

  // Radio setup
  out = -1;
  do {
    out = radio.begin();
    switch(out) {
      case RF24_BEGIN_SUCCESS: {
        serialPrint("\n>>> RF24 Successfully Initialized <<<\n");
        break;
      }
      case RF24_BEGIN_ERROR_CE_INVALID_PIN: {
        serialPrint("\n>>> RF24 INVALID CE PIN <<<\n", true);
        break;
      }
      case RF24_BEGIN_ERROR_CSN_INVALID_PIN: {
        serialPrint("\n>>> RF24 INVALID CSN PID <<<\n", true);
        break;
      }
      case RF24_BEGIN_ERROR_INIT_RADIO_BAD_CONFIG: {
        serialPrint("\n>>> RF24 BAD CONFIG <<<\n", true);
        break;
      }
      default: {
        serialPrint("\n>>> RF24 Impossible output <<<\n", true);
        break;
      }
      // Delay is embedded withing radio::_init_radio
    } 
  } while(out != RF24_BEGIN_SUCCESS);

  state.mode = OnPad;
}

// Loop globals
unsigned long last_time = 0;

void loop() {
  // Update Sensor Values
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  
  float currentTime = millis();
  state.dt = (currentTime - last_time) / 1000.0;

  state.ax = accel.acceleration.x;
  state.ay = accel.acceleration.y;
  state.az = accel.acceleration.z;

  state.gx = gyro.gyro.x * 180.0 / PI;
  state.gy = gyro.gyro.y * 180.0 / PI;
  state.gz = gyro.gyro.z * 180.0 / PI;

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
      ServoX.write(state.pidOutX);
      ServoY.write(state.pidOutY);
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
  if (currentTime - previous_log_time >= LOGGING_PERIOD_MS) {
    logToSD();
    previous_log_time = currentTime;
  }

  last_time = millis();
}
