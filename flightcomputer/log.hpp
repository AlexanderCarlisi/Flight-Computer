/// TODO:
/// Sector Logging
/// Cleanup, make this simpler

#ifndef LOG_H
#define LOG_H

#include <Arduino.h>

#define PRINT_TO_SERIAL   true
#define LOGGING_PERIOD_MS 1000

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

bool logCreateFile();

bool logInit();

void logStateToSD(FlightState& state);

void logCharToSD(char c);

void logErrToSD(String err);

void serialPrint(String message, bool err = false);

float getPrevLogTime();

void updatePrevLogTime();

#endif // LOG_H
