#ifndef LOG_H
#define LOG_H

// TODO: Split into multi files

///
/// Logging Framework
///
#define PRINT_TO_SERIAL   false
#define SD_CS_PIN         0
#define SD_LOG_PATH       String("/logs")
#define SD_LOG_FILENAME   String("log")
#define SD_LOG_EXT        String(".pat")
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
  File logsDir = SD.open(SD_LOG_PATH);
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


#endif // LOG_H