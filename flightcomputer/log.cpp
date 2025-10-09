/// TODO:
/// Implement Logging to sectors, instead of FAT SD.h
/// Get Metadata Epoch time from Radio

#include "log.hpp"
#include <SPI.h>

static float prev_log_ms = 0;

///
/// TODO: "Some commands take a time longer than NCR and it responds R1b"
///
byte send_cmd(uint8_t cmd, uint32_t arg = 0, uint8_t crc = 0x00) {
  // Send Command
  digitalWrite(LOG_CS, LOW);
  SPI.transfer(cmd);
  for (int i = 3; i >= 0; i--) { // MSB First
    SPI.transfer((arg >> (i * 8)) & 0xFF);
  }
  SPI.transfer(crc);

  // Read Response (8 clock cycles)
  byte res;
  for (int i = 0; i < 8; i++) {
    res = SPI.transfer(0xFF); // SPI Bus Synchronization
    if (!(res & LOG_REG_SYNC)) {
      digitalWrite(LOG_CS, HIGH);
      SPI.transfer(0xFF); // Dummy Byte
      break;
    }
  }
  return res;
}

bool log_init(LogMetadata lmd) {
  delay(10); // Need to wait at least 1ms before init
  pinMode(LOG_COPI, OUTPUT);
  pinMode(MOG_CPIO, INPUT);
  pinMode(LOG_CLK, OUTPUT);
  pinMode(LOG_CS, OUTPUT);
  
  // Power On
  // set clock rate to 100kHz-400kHz
  SPI.setClockDivider(SPI_CLOCK_DIV16); // 250kHz

  // set DI and CS high
  digitalWrite(LOG_CS, HIGH);
  digitalWrite(LOG_COPI, HIGH);
  
  // apply 74 or more clock pulses to SCLK
  // Card enters native operating mode and ready to accept command.
  for (int x = 0; x < 10; x++) { // 1 pulse = 1 bit
    SPI.transfer(0xFF);
  }
  
  // Software Reset
  // Send CMD0 with CS LOW, with valid CRC value
  byte r1 = send_cmd(LOG_CMD_RESET, 0x00, 0x95); // 0x95, working CRC value
  
  // Card enters SPI, responds R1, Idle State Bit set 0x01, CRC Disabled
  if (!(r1 & 0x01)) {
    Serial.print("\n>>>Idle Bit not Set<<<");
  }
  // TODO: Test ^^^^^^^

  // Initialization
  // In idle state, only accepts CMD{0,1,8,41,58,59}
  //
  // if we are outside of range 2.7-3.6V
  // *All cards work wihin 2.7-3.6V, so we're good in that case.
  // Check the Working Voltage Range with CMD58.
  // If the supply voltage is out of range, reject the card.
  // TODO: Don't supply over 3.6V so this isn't an issue
  //
  // CMD1 to initalize, continuously poll CMD1 until Idle State bit in R1
  // is cleared.
  // Timeout should be around 1 second
  // TODO: Add timeout
  do {
    r1 = send_cmd(LOG_CMD_INIT, 0x00);
    if (!(r1 & 0x01)) break;
  } while (true);

  // TODO:
  // Get the TRANS_SPEED field in the CSD Register, it indicates the Max Clock Speed.
  send_cmd(LOG_CMD_REGISTER, LOG_SECTOR_SIZE);
}

void log_state(FlightState& state) {
  
}

void log_err(String err) {
  
}

void log_print(String message, bool err = false) {
}

float log_time() {
  return prev_log_ms;
}

void log_time_update() {
  prev_log_ms = millis();
}
