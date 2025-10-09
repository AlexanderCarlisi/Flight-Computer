/// SDSC Reference
/// https://elm-chan.org/docs/mmc/mmc_e.html
///
/// Flow Chart
/// https://elm-chan.org/docs/mmc/rc/sdinit.png

#ifndef LOG_H
#define LOG_H

#include <Arduino.h>
#include <SPI.h>

#define PRINT_TO_SERIAL   true
#define LOGGING_PERIOD_MS 1000
#define LOG_START_SEQUENCE "$$$$$$$$"
#define LOG_STATE_SEQUENCE "&&&&&&&&"
#define LOG_ERROR_SEQUENCE "********"

///
/// SPI
///
/// When Writting:
/// Command Byte
/// <4 Bytes of Data>
/// CRC Byte (Disabled by Default)
/// Single Sector Write: MSB
/// 
#define LOG_COPI        0         // TODO:
#define LOG_CIPO        0         // TODO:
#define LOG_CLK         0         // TODO:
#define LOG_CS          0         // TODO:

/// SDSC Card Specs
#define LOG_SDSC_HZ     400000    // TODO: ?
#define LOG_BYTE_FIRST  MSBFIRST
#define LOG_SPI_MODE    0
#define LOG_SPI_SETTING SPISettings(LOG_SDSC_HZ, LOG_BYTE_FIRST, LOG_SPI_MODE)
#define LOG_SECTOR_SIZE 128

/// SDSC Cmds
#define LOG_CMD_RESET  0x40 | 0x00
#define LOG_CMD_INIT   0x40 | 0x01
#define LOG_CMD_SECTOR 0x40 | 0x10
#define LOG_CMD_WRITE  0x40 | 0x18

/// Registers
#define LOG_REG_SYNC      0x80
#define LOG_REG_ERR_PARAM 0x40
#define LOG_REG_ERR_ADDR  0x20
#define LOG_REG_ERR_ERASE 0x10
#define LOG_REG_ERR_CRC   0x08
#define LOG_REG_ERR_CMD   0x04
#define LOG_REG_ERASE     0x02
#define LOG_IDLE_STATE    0x01


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

typedef struct LogMetadata {
  long long epoch;
} LogMetaData;

bool log_init(LogMetadata lmd);

void log_state(FlightState& state);
void log_err(String err);
void log_print(String message, bool err = false);

float log_time();
void log_time_update();

#endif // LOG_H
