#include <main.h>
#include <utility/ATT.h>  // 045 F2: ATT.mtu() / ATT.connected() for the MTU instrumentation

BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); // Bluetooth® Low Energy LED Service

// Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

// Flight Logger Service
BLEService loggerService("F1706000-1234-5678-9ABC-DEF012345678");

// Control Characteristic - write commands (DL:filename, ERASE:ALL, LIST, NEXT)
BLEStringCharacteristic controlCharacteristic("F1706001-1234-5678-9ABC-DEF012345678", BLEWrite, 64);

// Data Characteristic - notify file chunks
BLECharacteristic dataCharacteristic("F1706002-1234-5678-9ABC-DEF012345678", BLERead | BLENotify, FLASH_LOGGER_CHUNK_SIZE);

// Status Characteristic - read/notify status (IDLE, DOWNLOADING, etc)
BLEStringCharacteristic statusCharacteristic("F1706003-1234-5678-9ABC-DEF012345678", BLERead | BLENotify, 64);

// File transfer state
static uint8_t transferBuffer[FLASH_LOGGER_CHUNK_SIZE];
static bool transferInProgress = false;
static bool bluetoothEnabled = true;
static uint32_t downloadCrc = 0;
static uint32_t downloadTotalBytes = 0;

// 045 F2 step 1 — download instrumentation (contracts/ble-download-v2.md). Protocol unchanged; these only measure.
static uint32_t dlChunks = 0;            // chunks sent this download
static uint32_t dlShortWrites = 0;       // notifies whose returned length < bytes read (ATT truncation to MTU-3, or no subscriber)
static int dlLastSentLen = -1;           // returned length of the most recent short notify
static unsigned long dlMaxLoopMs = 0;    // longest gap between blueToothLoop() calls while downloading (main-loop stalls)
static unsigned long dlMaxNextGapMs = 0; // longest gap between NEXT requests
static unsigned long dlLastLoopMs = 0;
static unsigned long dlLastNextMs = 0;
static unsigned long dlStartMs = 0;
static uint32_t dlReadUs = 0;            // total time in flashLoggerReadChunk this download
static uint32_t dlNotifyUs = 0;          // total time in the data notify (writeValue)

// The negotiated ATT MTU of the connected central. ArduinoBLE exposes MTU only by connection handle, so scan the low
// handles (Cordio allocates them from 0). 0 = no connected handle found.
static uint16_t currentAttMtu() {
  for (uint16_t h = 0; h < 16; h++) {
    if (ATT.connected(h)) return ATT.mtu(h);
  }
  return 0;
}

static void dlResetStats() {
  dlChunks = 0; dlShortWrites = 0; dlLastSentLen = -1;
  dlMaxLoopMs = 0; dlMaxNextGapMs = 0; dlReadUs = 0; dlNotifyUs = 0;
  dlLastLoopMs = dlLastNextMs = dlStartMs = millis();
}

// STAT:<offset>:<short_writes>:<max_loop_ms>:<max_next_gap_ms>:<read_ms_total>:<notify_ms_total>
static void dlFormatStat(char* buf, size_t n) {
  snprintf(buf, n, "STAT:%lu:%lu:%lu:%lu:%lu:%lu", (unsigned long)downloadTotalBytes, (unsigned long)dlShortWrites,
           dlMaxLoopMs, dlMaxNextGapMs, (unsigned long)(dlReadUs / 1000), (unsigned long)(dlNotifyUs / 1000));
}

// Reboot state for non-blocking reboot after erase
static unsigned long rebootScheduledTime = 0;
static bool rebootScheduled = false;


// CRC32 implementation (standard polynomial 0xEDB88320)
static uint32_t crc32Update(uint32_t crc, const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320 : 0);
    }
  }
  return crc;
}

void blueToothSetup()
{
  // begin initialization
  if (!BLE.begin())
  {
    logPrint(ERROR, "starting Bluetooth® Low Energy module failed!");

    // TODO don't hang here
    while (1)
      ;
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("XIAO-GP");
  // 045 F2: also the GAP device name, which BlueZ caches after a connection (it was the default "Arduino", so the page's
  // name filter missed the device on a host that had connected before).
  BLE.setDeviceName("XIAO-GP");
  BLE.setAdvertisedService(ledService);

  // add the characteristic to the LED service
  ledService.addCharacteristic(switchCharacteristic);

  // add LED service
  BLE.addService(ledService);

  // set the initial value for the characeristic:
  switchCharacteristic.writeValue(0);

  // Setup flight logger service
  loggerService.addCharacteristic(controlCharacteristic);
  loggerService.addCharacteristic(dataCharacteristic);
  loggerService.addCharacteristic(statusCharacteristic);
  BLE.addService(loggerService);

  // Initialize status
  statusCharacteristic.writeValue("IDLE");

  // start advertising
  BLE.advertise();

  logPrint(INFO, "BLE Peripheral (LED + FlightLogger)");
}

void blueToothSetEnabled(bool enabled)
{
  if (enabled == bluetoothEnabled) {
    return;
  }

  bluetoothEnabled = enabled;

  if (enabled) {
    BLE.advertise();
    logPrint(INFO, "BLE advertising enabled");
  } else {
    BLE.stopAdvertise();
    BLEDevice central = BLE.central();
    if (central && central.connected()) {
      logPrint(INFO, "BLE disconnecting active central due to arm state");
      central.disconnect();
    }
    logPrint(INFO, "BLE advertising disabled");
  }
}

// Forward declaration
static void transferNextChunk();

// Helper: process logger control commands
static void processLoggerCommand(const char* cmd) {
  logPrint(DEBUG, "Logger cmd: %s", cmd);

  if (strncmp(cmd, "LIST", 4) == 0) {
    // List files - send file info for each flight
    int fileCount = flashLoggerGetFileCount();
    uint32_t currentFlight = flashLoggerGetCurrentFlightNumber();
    logPrint(INFO, "Listing %d flight logs", fileCount);

    // Send current flight number
    char currentInfo[32];
    snprintf(currentInfo, sizeof(currentInfo), "CURRENT:%lu", (unsigned long)currentFlight);
    statusCharacteristic.writeValue(currentInfo);
    delay(2);  // Reduced from 10ms

    // Send each file as separate status notification
    for (int i = 0; i < fileCount; i++) {
      const char* filename = flashLoggerGetFileName(i);
      uint32_t size = flashLoggerGetFileSize(i);
      char fileInfo[64];
      snprintf(fileInfo, sizeof(fileInfo), "FILE:%s:%lu", filename, (unsigned long)size);
      statusCharacteristic.writeValue(fileInfo);
      delay(2);  // Reduced from 10ms
    }
    statusCharacteristic.writeValue("LIST_DONE");

  } else if (strncmp(cmd, "DL:", 3) == 0) {
    // Download file - start download and send ready status
    const char* filename = cmd + 3;

    // Parse flight number to find file size
    uint32_t flightNum = 0;
    uint32_t fileSize = 0;
    if (sscanf(filename, "flight_%lu.bin", (unsigned long*)&flightNum) == 1) {
      // Find this flight to get size
      int fileCount = flashLoggerGetFileCount();
      for (int i = 0; i < fileCount; i++) {
        const char* fname = flashLoggerGetFileName(i);
        if (fname && strcmp(fname, filename) == 0) {
          fileSize = flashLoggerGetFileSize(i);
          break;
        }
      }
    }

    logPrint(INFO, "Attempting download: %s (size: %lu bytes)", filename, (unsigned long)fileSize);

    if (fileSize == 0) {
      statusCharacteristic.writeValue("ERROR:File empty");
      logPrint(WARNING, "File is empty, nothing to download");
    } else if (flashLoggerStartDownload(filename)) {
      transferInProgress = true;
      downloadCrc = 0xFFFFFFFF;  // Initialize CRC
      downloadTotalBytes = 0;
      uint32_t filteredSize = flashLoggerGetActiveDownloadSize();
      if (filteredSize == 0) {
        filteredSize = fileSize;
      }
      // Send file size and chunk size so browser knows what to expect
      char statusMsg[64];
      snprintf(statusMsg, sizeof(statusMsg), "READY:%lu:%d", (unsigned long)filteredSize, FLASH_LOGGER_CHUNK_SIZE);
      statusCharacteristic.writeValue(statusMsg);
      dlResetStats();
      char mtuMsg[24];
      snprintf(mtuMsg, sizeof(mtuMsg), "MTU:%u", (unsigned)currentAttMtu());
      statusCharacteristic.writeValue(mtuMsg);
      logPrint(INFO, "Download ready: %s (%lu bytes), %s", filename, (unsigned long)filteredSize, mtuMsg);
      // Wait for browser to request chunks with NEXT commands
    } else {
      statusCharacteristic.writeValue("ERROR:Cannot start download");
      logPrint(ERROR, "Failed to start download");
    }

  } else if (strncmp(cmd, "NEXT", 4) == 0) {
    // Browser requests next chunk
    if (transferInProgress) {
      const unsigned long now = millis();
      if (now - dlLastNextMs > dlMaxNextGapMs) dlMaxNextGapMs = now - dlLastNextMs;
      dlLastNextMs = now;
      transferNextChunk();
    }

  } else if (strncmp(cmd, "ERASE:ALL", 9) == 0) {
    // Erase all logs
    statusCharacteristic.writeValue("ERASING");
    logPrint(INFO, "Erasing flash via BLE...");
    flashLoggerErase();
    statusCharacteristic.writeValue("ERASE_DONE");
    logPrint(INFO, "Erase complete, rebooting in 500ms...");
    // Schedule non-blocking reboot
    rebootScheduledTime = millis() + 500;
    rebootScheduled = true;

  } else {
    statusCharacteristic.writeValue("ERROR:Unknown command");
  }
}

// Send one chunk per NEXT request (simple request-response, no delays)
static void transferNextChunk() {
  const uint32_t r0 = micros();
  int bytesRead = flashLoggerReadChunk(transferBuffer, FLASH_LOGGER_CHUNK_SIZE);
  dlReadUs += micros() - r0;

  if (bytesRead < 0) {
    // Error
    logPrint(ERROR, "Flash read error during download (%lu bytes sent so far)",
             (unsigned long)downloadTotalBytes);
    statusCharacteristic.writeValue("ERROR:Read failed");
    flashLoggerStopDownload();
    transferInProgress = false;
    return;
  }

  if (bytesRead == 0) {
    // Complete - finalize CRC and send with status
    uint32_t finalCrc = downloadCrc ^ 0xFFFFFFFF;
    char statusMsg[64];
    char stat[64];
    dlFormatStat(stat, sizeof(stat));
    // DOWNLOAD_DONE:<crc>:<bytes>:<STAT fields> — the page reads fields 1 and 2; the rest is instrumentation.
    snprintf(statusMsg, sizeof(statusMsg), "DOWNLOAD_DONE:%08lX:%lu:%s",
             (unsigned long)finalCrc, (unsigned long)downloadTotalBytes, stat);
    const unsigned long ms = millis() - dlStartMs;
    logPrint(INFO, "Download complete: %lu bytes, CRC %08lX, %lu chunks in %lu ms (%lu B/s), %s, last short len %d",
             (unsigned long)downloadTotalBytes, (unsigned long)finalCrc, (unsigned long)dlChunks, ms,
             ms ? (unsigned long)(downloadTotalBytes * 1000UL / ms) : 0UL, stat, dlLastSentLen);
    statusCharacteristic.writeValue(statusMsg);
    flashLoggerStopDownload();
    transferInProgress = false;
    return;
  }

  // Update CRC with this chunk's data
  downloadCrc = crc32Update(downloadCrc, transferBuffer, bytesRead);
  downloadTotalBytes += bytesRead;
  // Send chunk via notification (no delay - pure sync request-response)
  const uint32_t w0 = micros();
  const int sent = dataCharacteristic.writeValue(transferBuffer, bytesRead);
  dlNotifyUs += micros() - w0;
  dlChunks++;
  if (sent != bytesRead) {  // 045 F2: measure truncation; step 2 makes this fail loud (ERR:SHORT_NOTIFY)
    dlShortWrites++;
    dlLastSentLen = sent;
  }
  if (dlChunks % 16 == 0) {
    char stat[64];
    dlFormatStat(stat, sizeof(stat));
    statusCharacteristic.writeValue(stat);
  }
}

// Non-blocking polling loop for BT work
void blueToothLoop()
{
  static BLEDevice lastCentral;
  static bool wasConnected = false;

  // Check for scheduled reboot (non-blocking)
  if (rebootScheduled && millis() >= rebootScheduledTime) {
    NVIC_SystemReset();
  }

  // 045 F2: main-loop period while a download runs (a blocking MSP poll shows up here)
  if (transferInProgress) {
    const unsigned long now = millis();
    if (now - dlLastLoopMs > dlMaxLoopMs) dlMaxLoopMs = now - dlLastLoopMs;
    dlLastLoopMs = now;
  }

  // Get current central
  BLEDevice central = BLE.central();

  // Check for new connection
  if (central && !wasConnected)
  {
    logPrint(INFO, "Connected to central: %s, MTU %u", central.address().c_str(), (unsigned)currentAttMtu());
    lastCentral = central;
    wasConnected = true;
  }

  // If we have an active connection, service it (non-blocking)
  if (central && central.connected())
  {
    // Handle LED control
    if (switchCharacteristic.written())
    {
      if (switchCharacteristic.value())
      {
        analogWrite(BLUE_PIN, 0);
        logPrint(INFO, "LED on");
      }
      else
      {
        analogWrite(BLUE_PIN, 255);
        logPrint(INFO, "LED off");
      }
    }

    // Handle logger control commands (including NEXT for chunk requests)
    if (controlCharacteristic.written())
    {
      String cmd = controlCharacteristic.value();
      processLoggerCommand(cmd.c_str());
    }
  }
  else if (wasConnected)
  {
    // Disconnected
    logPrint(DEBUG, "Disconnected from central");
    wasConnected = false;

    // Clean up any in-progress transfer
    if (transferInProgress) {
      char stat[64];
      dlFormatStat(stat, sizeof(stat));
      logPrint(WARNING, "Disconnected mid-download after %lu ms: %s", millis() - dlStartMs, stat);
      flashLoggerStopDownload();
      transferInProgress = false;
    }
  }
}

// --- 045 F2: wired log read over the USB serial console ----------------------------------------------------------------
// The byte reference for checking a BLE download (tasks.md T016), and a field fallback with a laptop.
//   LIST          -> "FILE:<name>:<size>" lines, then "LIST_DONE"
//   DUMP:<name>   -> "DUMP_BEGIN:<name>:<size>\n", <size> raw bytes, "\nDUMP_END:<crc32 hex>:<bytes>\n"
// Refused while armed, or while a BLE download holds the flash. The dump is one tight loop (no logPrint inside it), so no
// console text can land in the binary stream; it feeds the watchdog per chunk. specs/045-m2-tracking/xiao_serial_dump.py
// is the host side.
static void serialDump(const char* filename) {
  if (state.isArmed()) { Serial.println("DUMP_ERR:armed"); return; }
  if (transferInProgress) { Serial.println("DUMP_ERR:ble download in progress"); return; }
  if (!flashLoggerStartDownload(filename)) { Serial.println("DUMP_ERR:cannot start (unknown file or flash busy)"); return; }
  const uint32_t size = flashLoggerGetActiveDownloadSize();
  Serial.print("DUMP_BEGIN:"); Serial.print(filename); Serial.print(":"); Serial.println((unsigned long)size);
  Serial.flush();
  static uint8_t buf[FLASH_LOGGER_CHUNK_SIZE];
  uint32_t crc = 0xFFFFFFFF, total = 0;
  uint32_t readUs = 0, writeUs = 0;
  const uint32_t t0 = millis();
  for (;;) {
    if (!Serial) { flashLoggerStopDownload(); return; }  // host closed the port: stop, don't feed a dead pipe
    uint32_t a = micros();
    const int n = flashLoggerReadChunk(buf, sizeof(buf));
    readUs += micros() - a;
    if (n < 0) { flashLoggerStopDownload(); Serial.print("\nDUMP_ERR:read failed at "); Serial.println((unsigned long)total); return; }
    if (n == 0) break;
    crc = crc32Update(crc, buf, n);
    total += n;
    a = micros();
    Serial.write(buf, n);
    writeUs += micros() - a;
    faultGuardFeed();
  }
  flashLoggerStopDownload();
  char tail[96];
  // DUMP_END:<crc32>:<bytes>:<total ms>:<flash read ms>:<usb write ms> — the last three are instrumentation (045 F2)
  snprintf(tail, sizeof(tail), "\nDUMP_END:%08lX:%lu:%lu:%lu:%lu", (unsigned long)(crc ^ 0xFFFFFFFF), (unsigned long)total,
           (unsigned long)(millis() - t0), (unsigned long)(readUs / 1000), (unsigned long)(writeUs / 1000));
  Serial.println(tail);
  Serial.flush();
}

void serialConsoleLoop() {
  static char line[64];
  static size_t len = 0;
  while (Serial.available() > 0) {
    const int c = Serial.read();
    if (c < 0) break;
    if (c == '\r') continue;
    if (c != '\n') {
      if (len < sizeof(line) - 1) line[len++] = (char)c;
      continue;
    }
    line[len] = '\0';
    len = 0;
    if (strcmp(line, "LIST") == 0) {
      const int count = flashLoggerGetFileCount();
      for (int i = 0; i < count; i++) {
        Serial.print("FILE:"); Serial.print(flashLoggerGetFileName(i)); Serial.print(":");
        Serial.println((unsigned long)flashLoggerGetFileSize(i));
      }
      Serial.println("LIST_DONE");
    } else if (strncmp(line, "DUMP:", 5) == 0) {
      serialDump(line + 5);
    } else if (strcmp(line, "QPROF") == 0) {
      if (state.isArmed()) { Serial.println("QPROF_ERR:armed"); }
      else { char out[480]; flashLoggerProfileRead(out, sizeof(out)); Serial.println(out); }
    } else if (line[0] != '\0') {
      Serial.print("CONSOLE_ERR:unknown command: "); Serial.println(line);
    }
  }
}
