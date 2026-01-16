/// TODO List
/// - Logging
/// - Radio Communication
/// - Test Swapping to ESP32
///   - Servos
///   - Gyro
///   - PID Loop
///   - Serial Mode Change
///   - Logging Framework

#include <ESP32Servo.h>
#include <RF24.h>
#include <RF24_config.h>
#include <nRF24L01.h>
#include <printf.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>

/// DEFINITIONS
#define MODE_CHANGE         1   // 0 = Velocity (automatic), 1 = Serial Input
#define HALT_ON_INIT_ERR    0   // Whether to halt execution when an error occurs in Initialization or not

#define LOG_TO_SERIAL       1   // Whether log functions should additionally print to Serial or not
#define LOG_TO_SD           1   // Whether log functions should write to the SD Card or not, should be disabled when debugging in Loops
#define SERIAL_STATE        1   // Whether to Print the FlightState to Serial or not
#define LOG_PERIOD_MS       1000

#define RADIO_IS_CLOSE      1   // If the RF24 Radios are close together, supply less power for interference reasons
#define RF_PIPE   0
#define RF_CHNL   110
#define RF_TX     {'P', 'A', 'T', '0', '1', '\0'}
#define RF_RX     {'P', 'A', 'T', '0', '2', '\0'}

// Radio VSPI Bus
#define RADIO_CE  4
#define RADIO_CSN 5
#define RADIO_SPI_SCK   18
#define RADIO_SPI_MISO  19
#define RADIO_SPI_MOSI  23

// #define MPU_ADDR    0x68   // Has default value in Lib, if that doesnt work, set manually here
#define MPU_ACCEL_RANGE   MPU6050_RANGE_16_G
#define MPU_GYRO_RANGE    MPU6050_RANGE_500_DEG
#define MPU_FILTER_BAND   MPU6050_BAND_21_HZ
#define BME_ADDR    0x76
#define SERVO_X_PIN 0
#define SERVO_Y_PIN 0

#define DESIRED_PITCH   -90   // Defines what is Straight up for the Servos
#define DESIRED_ROLL    0     // ^
#define PIDCONST_P      2
#define PIDCONST_I      0.05
#define PIDCONST_D      0.1
#define PID_PERIOD      20    // Miliseconds between PID Updates

#define FREEFALL_THRESHOLD  2.0   // m/s/s checks accelerometer
#define DESCENT_THRESHOLD   0.5   // Altitude velocity threshold to be considered at Apogee
#define APOGEE_COUNT        5     // Consecutive ticks at APOGEE until we trust it
#define ABORT_DEGREES       90    // Call Abort Function at this value on Gyro for X||Y||Z
#define GRAVITY_FIELD       9.81  // Earth

typedef enum Mode {
  PreInit,        // Initializing Code
  OnPad,          // OnPad, Awaiting Launch
  PoweredFlight,  // Launched: Servoing
  Coast           // Deployed Parachute / Aborted
} Mode;

typedef struct FlightState {
  float dt, ax, ay, az,
        gx, gy, gz,
        temperature, pressure, altitude,
        accPitch, accRoll,
        pitch, roll, yaw,
        pidOutX, pidOutY,
        pidIPitch, pidIRoll,
        pidPrevErrPitch, pidPrevErrRoll,
        pidErrPitch, pidErrRoll,
        servoAngleX, servoAngleY;
  int descent_count;
  bool parachute_deployed;
  bool abort;
  Mode mode;
} FlightState;

void halt();
void mpu_init(Adafruit_MPU6050& mpu);
void bme_init(Adafruit_BME280& bme);
void radio_init(RF24& radio);
void log_init();

void log(String message);
void log_err(String message);
void log_state(FlightState& state);

void radio_establish_connection(RF24& radio);

void deploy_parachute();
void abort();
void mode_change_serial(Mode& mode);
void mode_change_velocity(Mode& mode);

/** 
 * @brief Perform the PID Calculation on the provided values.
 * @note integral, preverr, err are Pointers, they will be populated with new values.
 */
float pid(float setpoint, float measurement, float dt, float& integral, float& prevErr, float& err);

/// Globals
const byte RADIO_TX_BYTES[] = RF_TX;
const byte RADIO_RX_BYTES[] = RF_RX;

unsigned long prev_timestamp = 0;
unsigned long prev_log_timestamp = 0;
unsigned long prev_pid_timestamp = 0;
float previous_altitude = 0;
bool logger_initialized = false;
bool mpu_initialized = false;
bool bme_initialized = false;
bool radio_initialized = false;

FlightState flight_state;
Adafruit_MPU6050 mpu6050;
Adafruit_BME280 bme280;
RF24 rf24_radio(RADIO_CE, RADIO_CSN);
Servo servo_x;
Servo servo_y;

void setup() {
  Serial.begin(115200);
  log("Serial Connected");
  log("Begin Setup");
  flight_state.mode = PreInit;

  log_init();
  log("Logger ✔");

  mpu_init(mpu6050);
  log("MPU6050 ✔");

  bme_init(bme280);
  log("BME280 ✔");

  servo_x.attach(SERVO_X_PIN, 500, 2400);
  servo_y.attach(SERVO_Y_PIN, 500, 2400);
  log("Servos ✔");

  log("Begin SPI");
  SPI.begin(RADIO_SPI_SCK, RADIO_SPI_MISO, RADIO_SPI_MOSI, RADIO_CSN);
  log("Radio Setup");
  radio_init(rf24_radio);
  log("Radio Initialized");
  radio_establish_connection(rf24_radio);
  log("Radio ✔");

  log("Setup Complete, FlightState => OnPad");
  flight_state.mode = OnPad;
}

void loop() {
  unsigned long currentTime = millis();
  flight_state.dt = (currentTime - prev_timestamp) / 1000.0;
  prev_timestamp = currentTime;
  
  sensors_event_t a, g, temp;
  mpu6050.getEvent(&a, &g, &temp);

  // m/s/s
  flight_state.ax = a.acceleration.x;
  flight_state.ay = a.acceleration.y;
  flight_state.az = a.acceleration.z;

  // radians to degrees
  flight_state.gx = g.gyro.x * 180.0 / PI;
  flight_state.gy = g.gyro.y * 180.0 / PI;
  flight_state.gz = g.gyro.z * 180.0 / PI;

  flight_state.temperature = bme280.readTemperature();
  flight_state.pressure = bme280.readPressure() / 100.0F; // hPa
  flight_state.altitude = bme280.readAltitude(1013.25);   // Sea-level pressure (hPa)

  flight_state.accPitch = atan2(-flight_state.ax, flight_state.az) * 180.0 / PI;
  flight_state.accRoll  = atan2(flight_state.ay, flight_state.az) * 180.0 / PI;
  
  // Complementary filter constant between the Gyro and Accelerometer
  float alpha = 0.98; // TODO: may need tuning

  flight_state.pitch = alpha * (flight_state.pitch + flight_state.gy * flight_state.dt) + (1 - alpha) * flight_state.accPitch;
  flight_state.roll = alpha * (flight_state.roll + flight_state.gx * flight_state.dt) + (1 - alpha) * flight_state.accRoll;
  flight_state.yaw += flight_state.gz * flight_state.dt;

  // Servo output
  if (currentTime - prev_pid_timestamp >= PID_PERIOD) {
    flight_state.pidOutX = pid(DESIRED_PITCH, flight_state.pitch, flight_state.dt, flight_state.pidIPitch, flight_state.pidPrevErrPitch, flight_state.pidErrPitch);
    flight_state.pidOutY = pid(DESIRED_ROLL , flight_state.roll, flight_state.dt, flight_state.pidIRoll, flight_state.pidPrevErrRoll, flight_state.pidErrRoll);

    flight_state.servoAngleX = map(flight_state.pidOutX, -90, 90, 0, 180);
    flight_state.servoAngleY = map(flight_state.pidOutY, -90, 90, 0, 180);

    flight_state.servoAngleX = constrain(flight_state.servoAngleX, 0, 180);
    flight_state.servoAngleY = constrain(flight_state.servoAngleY, 0, 180);

    prev_pid_timestamp = currentTime;
  }
  
  // mode switch logic
  if (MODE_CHANGE == 0) {
    mode_change_serial(flight_state.mode);
  } else {
    mode_change_velocity(flight_state);
  }
  
  // Perform actions based on mode
  switch(flight_state.mode) {
    case OnPad: {
      break;
    }
    case PoweredFlight: {
      servo_x.write(flight_state.servoAngleX);
      servo_y.write(flight_state.servoAngleY);
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
  if (currentTime - prev_log_timestamp >= LOG_PERIOD_MS) {
    log_state(flight_state);
    prev_log_timestamp = currentTime;
  }
}

void halt() {
  while (1) {
    delay(10);
  }
}

void log_init() { // TODO: 
  // if (HALT_ON_INIT_ERR) {
  //   do {

  //   } while (1);
  //   logger_initialized = true;
  // }
}

void mpu_init(Adafruit_MPU6050& mpu) {
  mpu_initialized = mpu.begin();
  if (!mpu_initialized) {
    log_err("MPU Initialization Failed");
    if (HALT_ON_INIT_ERR) halt();
  } else {
    mpu.setAccelerometerRange(MPU_ACCEL_RANGE);
    mpu.setGyroRange(MPU_GYRO_RANGE);
    mpu.setFilterBandwidth(MPU_FILTER_BAND);
  }
}

void bme_init(Adafruit_BME280& bme) {
  bme_initialized = bme.begin();
  if (!bme_initialized) {
    log_err("BME Initialization Failed");
    if (HALT_ON_INIT_ERR) halt();
  }
}

void radio_init(RF24& radio) {
  radio_initialized = radio.begin();
  if (!radio_initialized) {
    log_err("Radio Initialization Failed");
    if (HALT_ON_INIT_ERR) halt();

  } else {
    if (RADIO_IS_CLOSE)
      radio.setPALevel(RF24_PA_LOW); // MAX is default
    radio.setChannel(RF_CHNL);
    radio.stopListening(RADIO_TX_BYTES); // set writes on Pipe 0, also puts in writting mode
    radio.openReadingPipe(1, RADIO_RX_BYTES); // reads on Pipe 1
    // radio.startListening() puts it in listening mode / RX Mode
  }
}

void log(String message) {
  if (LOG_TO_SERIAL) {
    Serial.println(message);
  }
  if (LOG_TO_SD) {

  }
}

void log_err(String message) {
  message = "\n<ERR> " + message + " <ERR>\n";
  log(message);
}

void log_state(FlightState& state) {
  if (SERIAL_STATE) {
    Serial.print("DeltaTime: "); Serial.print(state.dt);
    Serial.print("accelX: "); Serial.print(state.ax);
    Serial.print("accelY: "); Serial.print(state.ay);
    // todo: implement the rest
  }
  if (LOG_TO_SD) {
    // Log bytes of state
  }
}

void radio_establish_connection(RF24& radio) {
  byte payload[] = "Hello World!";
  radio.stopListening(RADIO_TX_BYTES);
  bool report = radio.write(&payload, sizeof(payload));

  if (report) {
    log("Transmission Successful");

  } else {
    log_err("Transmission failed or timed out");
    if (HALT_ON_INIT_ERR) halt();
  }
}

void deploy_parachute() { }
void abort() {
  deploy_parachute();
}

void mode_change_serial(Mode& mode) {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Read full line until Enter
    input.trim(); // Remove spaces/newlines

    if (input.length() > 0 && input.charAt(0) >= '0' && input.charAt(0) <= '3') {
      int modeNumber = input.toInt();
      if (modeNumber >= 0 && modeNumber <= 3 && ((Mode) modeNumber) != mode) {
        mode = (Mode) modeNumber;
        Serial.print("Flight mode set to: ");
        switch (mode) {
          case PreInit:        Serial.println("PreInit"); break;
          case OnPad:          Serial.println("OnPad"); break;
          case PoweredFlight:  Serial.println("PoweredFlight"); break;
          case Coast:          Serial.println("Coast"); break;
        }
        log("Mode Change");
      } else {
        Serial.println("Invalid mode. Use: 0=PreInit, 1=OnPad, 2=PoweredFlight, 3=Coast");
        log_err("Invalid Mode Input from Serial");
      }
    }
  }
}

void mode_change_velocity(FlightState& state) {
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

    previous_altitude = state.altitude;
  }

  // Check conditions for Abort Command
  // TODO: Tune these conditions
  if (!state.abort && (abs(state.yaw) >= ABORT_DEGREES || abs(state.pitch) >= ABORT_DEGREES)) {
    abort();
    state.abort = true;
  }
}

float pid(float setpoint, float measurement, float dt, float& integral, float& prevErr, float& err) {
  prevErr = err;
  err = setpoint - measurement;
  integral += err;
  return PIDCONST_P * err + PIDCONST_I * integral + PIDCONST_D * (err - prevErr) / dt;
}
