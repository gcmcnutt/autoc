#pragma once

#include <Arduino.h>

// Flash logger configuration
#define FLASH_LOGGER_BUFFER_SIZE 4096  // 4KB RAM buffer
#define FLASH_LOGGER_CHUNK_SIZE 240    // BLE transfer chunk size (matches BLE MTU)
#define FLASH_MAX_FILES 999            // flight_001.bin to flight_999.bin

// Flash logger states
enum FlashLoggerState {
  FLASH_IDLE,
  FLASH_DOWNLOADING,
  FLASH_ERASING
};

// Flash logger API
void flashLoggerInit();
// 039 US3: flash carries ONLY the versioned binary flight-log records
// (flight_log_format.h); console text never goes to flash (FR-014). Appends
// one whole record; returns false on buffer pressure / flash-full / suspended
// (caller counts drops — FR-008: never stall the tick, never split a record).
bool flashLoggerWriteBinary(const void* data, size_t len);
void flashLoggerFlushCheck();
bool flashLoggerBeginFlight();
void flashLoggerEndFlight();
void flashLoggerErase();
bool flashLoggerIsFull();
bool flashLoggerIsSuspended();

// File transfer API (for BLE)
int flashLoggerGetFileCount();
const char* flashLoggerGetFileName(int index);
uint32_t flashLoggerGetFileSize(int index);
uint32_t flashLoggerGetCurrentFlightNumber();
bool flashLoggerStartDownload(const char* filename);
int flashLoggerReadChunk(uint8_t* buffer, size_t maxLen);
void flashLoggerStopDownload();
FlashLoggerState flashLoggerGetState();
bool flashLoggerHasPendingData();
uint32_t flashLoggerGetActiveDownloadSize();
