#include "log.hpp"
#include <Arduino.h>

unsigned long previous_log_time = 0;
String log_filename = "";

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


void logStateToSD(FlightState& state) {
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
  File logFile = SD.open(log_filename);
  logFile.print(err);
  logFile.close();
}


void serialPrint(String message, bool err = false) {
  if (PRINT_TO_SERIAL || err) Serial.print(message);
  if (err) logErrToSD(message);
}

float getPrevLogTime() {
  return previous_log_time;
}

float updatePrevLogTime() {
  previous_log_time = millis();
}
