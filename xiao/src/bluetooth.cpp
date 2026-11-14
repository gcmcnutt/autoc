#include <main.h>
#include <utility/ATT.h>  // ATT.mtu() / ATT.connected(): the negotiated MTU sizes the v2 chunks
#include "ble_dl2.h"      // 045 F2: the v2 send window (pure logic; tests/ble_dl2_tests.cc)

// Flight-log download over BLE, protocol v2 (specs/045-m2-tracking/contracts/ble-download-v2.md, tasks.md T019–T022).
// Measured 2026-11-12 (research.md R2): v1 was strict stop-and-wait, one 240 B chunk per ~97.5 ms (two connection intervals),
// ~2.4 kB/s. v2 keeps up to `window` offset-tagged chunks in flight, ACKed cumulatively by the page, with go-back-N on a gap,
// resume from any offset, an inactivity abort, and a loud stop on a truncated notify. v1 (DL: / NEXT) is gone (clean cut).
//   page -> control: DL2:<file>:<offset>:<window>   ACK:<offset>   ABORT   LIST   ERASE:ALL
//   fw   -> status:  READY2:<size>:<chunk>:<mtu>   STAT:...   DONE2:<size>:<crc32>:<STAT...>   ERR:<reason>:<offset>
//   fw   -> data:    [u32 offset LE][payload] per notification

BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); // Bluetooth® Low Energy LED Service

// Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

// Flight Logger Service
BLEService loggerService("F1706000-1234-5678-9ABC-DEF012345678");

static constexpr uint32_t kDl2MaxChunk = 240;  // payload cap = MTU 247 - 3 (ATT) - 4 (offset)
static constexpr uint32_t kDl2MaxNotify = dl2::kHeaderBytes + kDl2MaxChunk;

// Control: commands. Write-without-response too, so the page's ACKs don't each cost a round trip.
BLEStringCharacteristic controlCharacteristic("F1706001-1234-5678-9ABC-DEF012345678", BLEWrite | BLEWriteWithoutResponse, 64);

// Data: [u32 offset][payload] notifications
BLECharacteristic dataCharacteristic("F1706002-1234-5678-9ABC-DEF012345678", BLERead | BLENotify, kDl2MaxNotify);

// Status Characteristic - read/notify status (IDLE, READY2, STAT, DONE2, ERR)
BLEStringCharacteristic statusCharacteristic("F1706003-1234-5678-9ABC-DEF012345678", BLERead | BLENotify, 64);

static bool bluetoothEnabled = true;

// v2 transfer state
static dl2::Sender dlTx;                 // dlTx.active == a download holds the flash
static uint8_t dlBuf[kDl2MaxNotify];
static uint32_t dlReadPos = 0;           // file offset of the flash read cursor
// instrumentation (STAT)
static uint32_t dlNotifies = 0;
static unsigned long dlMaxLoopMs = 0, dlLastLoopMs = 0, dlStartMs = 0;
static uint32_t dlReadUs = 0, dlNotifyUs = 0;

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

// The negotiated ATT MTU of the connected central. ArduinoBLE exposes MTU only by connection handle, so scan the low
// handles (Cordio allocates them from 0). 0 = no connected handle found.
static uint16_t currentAttMtu() {
  for (uint16_t h = 0; h < 16; h++) {
    if (ATT.connected(h)) return ATT.mtu(h);
  }
  return 0;
}

bool blueToothDownloadActive() { return dlTx.active; }

// STAT:<acked offset>:<notifies>:<rewinds>:<max_loop_ms>:<read_ms_total>:<notify_ms_total>
static void dlFormatStat(char* buf, size_t n) {
  snprintf(buf, n, "STAT:%lu:%lu:%lu:%lu:%lu:%lu", (unsigned long)dlTx.ackOff, (unsigned long)dlNotifies,
           (unsigned long)dlTx.rewinds, dlMaxLoopMs, (unsigned long)(dlReadUs / 1000), (unsigned long)(dlNotifyUs / 1000));
}

static void dlStop(const char* status) {
  flashLoggerStopDownload();
  dlTx.active = false;
  if (status) statusCharacteristic.writeValue(status);
}

static void dlFail(const char* reason, uint32_t off) {
  char msg[64];
  snprintf(msg, sizeof(msg), "ERR:%s:%lu", reason, (unsigned long)off);
  logPrint(WARNING, "BLE download stopped: %s", msg);
  dlStop(msg);
}

// Whole-file CRC32, a separate pass over the flash (resume and go-back-N mean the bytes were sent out of order).
static bool dlFileCrc(uint32_t* crcOut) {
  if (!flashLoggerSeekDownload(0)) return false;
  uint32_t crc = 0xFFFFFFFF;
  for (;;) {
    const int n = flashLoggerReadChunk(dlBuf, kDl2MaxChunk);
    if (n < 0) return false;
    if (n == 0) break;
    crc = crc32Update(crc, dlBuf, n);
  }
  *crcOut = crc ^ 0xFFFFFFFF;
  return true;
}

static void dlFinish() {
  uint32_t crc = 0;
  if (!dlFileCrc(&crc)) { dlFail("READ", dlTx.size); return; }
  char stat[64], msg[64];
  dlFormatStat(stat, sizeof(stat));
  snprintf(msg, sizeof(msg), "DONE2:%lu:%08lX", (unsigned long)dlTx.size, (unsigned long)crc);
  const unsigned long ms = millis() - dlStartMs;
  logPrint(INFO, "BLE download done: %lu bytes, CRC %08lX, %lu ms (%lu B/s), %s", (unsigned long)dlTx.size,
           (unsigned long)crc, ms, ms ? (unsigned long)(dlTx.size * 1000UL / ms) : 0UL, stat);
  statusCharacteristic.writeValue(stat);
  dlStop(msg);
}

// DL2:<file>:<offset>:<window>
static void dlStart(const char* args) {
  char file[40];
  const char* c1 = strchr(args, ':');
  if (!c1 || (size_t)(c1 - args) >= sizeof(file)) { statusCharacteristic.writeValue("ERR:BADCMD:0"); return; }
  memcpy(file, args, c1 - args);
  file[c1 - args] = '\0';
  char* end = nullptr;
  const uint32_t offset = strtoul(c1 + 1, &end, 10);
  const uint32_t window = (end && *end == ':') ? strtoul(end + 1, nullptr, 10) : 8;

  if (dlTx.active) dlStop(nullptr);  // a new DL2 replaces any transfer in progress
  const uint16_t mtu = currentAttMtu();
  uint32_t chunk = dl2::chunkForMtu(mtu);
  if (chunk > kDl2MaxChunk) chunk = kDl2MaxChunk;
  if (chunk == 0) { dlFail("MTU", mtu); return; }
  if (!flashLoggerStartDownload(file)) { dlFail("NOFILE", 0); return; }
  const uint32_t size = flashLoggerGetActiveDownloadSize();
  if (size == 0 || offset > size || !flashLoggerSeekDownload(offset)) { dlFail("RANGE", offset); return; }
  dlReadPos = offset;
  dlNotifies = 0; dlMaxLoopMs = 0; dlReadUs = 0; dlNotifyUs = 0;
  dlStartMs = dlLastLoopMs = millis();
  dl2::start(dlTx, size, chunk, offset, window, millis());
  char msg[64];
  snprintf(msg, sizeof(msg), "READY2:%lu:%lu:%u", (unsigned long)size, (unsigned long)chunk, (unsigned)mtu);
  statusCharacteristic.writeValue(msg);
  logPrint(INFO, "BLE download: %s from %lu of %lu B, chunk %lu, window %lu, MTU %u", file, (unsigned long)offset,
           (unsigned long)size, (unsigned long)chunk, (unsigned long)dlTx.window, (unsigned)mtu);
  if (!dlTx.active) dlFinish();  // resume at the end: nothing to send
}

// Fill the window: read each chunk at its offset and notify it. A short notify (truncated by the MTU, or no subscriber)
// stops the transfer loudly rather than corrupting it.
static void dlService() {
  if (!dlTx.active) return;
  const unsigned long now = millis();
  if (now - dlLastLoopMs > dlMaxLoopMs) dlMaxLoopMs = now - dlLastLoopMs;
  dlLastLoopMs = now;
  if (dl2::poll(dlTx, now) == dl2::Event::Inactive) { dlFail("INACTIVE", dlTx.ackOff); return; }
  uint32_t off, len;
  while (dl2::next(dlTx, &off, &len)) {
    if (off != dlReadPos) {
      if (!flashLoggerSeekDownload(off)) { dlFail("SEEK", off); return; }
      dlReadPos = off;
    }
    uint32_t t = micros();
    const int n = flashLoggerReadChunk(dlBuf + dl2::kHeaderBytes, len);
    dlReadUs += micros() - t;
    if (n != (int)len) { dlFail("READ", off); return; }
    dlReadPos += len;
    dlBuf[0] = off & 0xFF; dlBuf[1] = (off >> 8) & 0xFF; dlBuf[2] = (off >> 16) & 0xFF; dlBuf[3] = (off >> 24) & 0xFF;
    t = micros();
    const int sent = dataCharacteristic.writeValue(dlBuf, len + dl2::kHeaderBytes);
    dlNotifyUs += micros() - t;
    if (sent != (int)(len + dl2::kHeaderBytes)) { dlFail("SHORT_NOTIFY", off); return; }
    if (++dlNotifies % 64 == 0) {
      char stat[64];
      dlFormatStat(stat, sizeof(stat));
      statusCharacteristic.writeValue(stat);
    }
  }
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
  // 045 F2: ask the central for a short connection interval (7.5–15 ms, units of 1.25 ms): more connection events per second
  // carry more of the v2 window. The central decides; a long interval just means fewer events.
  BLE.setConnectionInterval(6, 12);
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

  logPrint(INFO, "BLE Peripheral (LED + FlightLogger, download v2)");
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
    if (dlTx.active) dlStop(nullptr);
    BLE.stopAdvertise();
    BLEDevice central = BLE.central();
    if (central && central.connected()) {
      logPrint(INFO, "BLE disconnecting active central due to arm state");
      central.disconnect();
    }
    logPrint(INFO, "BLE advertising disabled");
  }
}

// Helper: process logger control commands
static void processLoggerCommand(const char* cmd) {
  if (strncmp(cmd, "ACK:", 4) == 0) {  // the hot path: no logging
    if (dlTx.active && dl2::onAck(dlTx, strtoul(cmd + 4, nullptr, 10), millis()) == dl2::Event::Done) dlFinish();
    return;
  }
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

  } else if (strncmp(cmd, "DL2:", 4) == 0) {
    dlStart(cmd + 4);

  } else if (strncmp(cmd, "ABORT", 5) == 0) {
    if (dlTx.active) {
      logPrint(INFO, "BLE download aborted by the page at %lu", (unsigned long)dlTx.ackOff);
      dlStop("ABORTED");
    }

  } else if (strncmp(cmd, "ERASE:ALL", 9) == 0) {
    if (dlTx.active) { statusCharacteristic.writeValue("ERR:BUSY:0"); return; }
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

// Non-blocking polling loop for BT work
void blueToothLoop()
{
  static BLEDevice lastCentral;
  static bool wasConnected = false;

  // Check for scheduled reboot (non-blocking)
  if (rebootScheduled && millis() >= rebootScheduledTime) {
    NVIC_SystemReset();
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

    // Handle logger control commands (DL2, ACK, ABORT, LIST, ERASE:ALL)
    if (controlCharacteristic.written())
    {
      String cmd = controlCharacteristic.value();
      processLoggerCommand(cmd.c_str());
    }
    dlService();
  }
  else if (wasConnected)
  {
    // Disconnected
    logPrint(DEBUG, "Disconnected from central");
    wasConnected = false;

    // The page keeps its buffer and resumes with DL2:<file>:<offset>; here, just free the flash.
    if (dlTx.active) {
      char stat[64];
      dlFormatStat(stat, sizeof(stat));
      logPrint(WARNING, "Disconnected mid-download after %lu ms: %s", millis() - dlStartMs, stat);
      dlStop(nullptr);
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
  if (dlTx.active) { Serial.println("DUMP_ERR:ble download in progress"); return; }
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
    } else if (strcmp(line, "RESET") == 0) {
      // 045 bench: the same as pressing the reset button, reachable only over USB (disconnected in flight). Used for the
      // download-resume-across-a-reboot test and the H3 xiao-reboot safety test.
      Serial.println("RESETTING");
      Serial.flush();
      delay(50);
      NVIC_SystemReset();
    } else if (strcmp(line, "QPROF") == 0) {
      if (state.isArmed()) { Serial.println("QPROF_ERR:armed"); }
      else { char out[480]; flashLoggerProfileRead(out, sizeof(out)); Serial.println(out); }
    } else if (line[0] != '\0') {
      Serial.print("CONSOLE_ERR:unknown command: "); Serial.println(line);
    }
  }
}
