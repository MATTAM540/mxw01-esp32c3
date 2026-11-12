/*
 * esp32c3_mxw01.ino
 *
 * MXW01 BLE thermal printer driver for ESP32-C3 Super Mini (NimBLE-Arduino 2.x).
 *
 * Every command below was verified against a live MXW01 (fw 1.9.3.1.2):
 *
 *   frame : 22 21 <cmd> 00 <len_lo> <len_hi> <payload...> <crc8(payload)> FF
 *   print : A2(intensity) -> A9(lines_lo, lines_hi, 0x30, mode)
 *           -> raw 48-byte rows on AE03 -> AD flush -> wait 0xAA
 *
 * Rows are 384 px wide, 1bpp, LSB-first inside each byte (pixel x -> bit x&7).
 *
 * Serial commands (115200):
 *   p        print the built-in self test page
 *   t <txt>  print text (uppercase mapped, auto word wrap)
 *   s        query status (state / battery / temperature / error)
 *   b        query battery percentage
 *   v        query firmware version
 *   i <0-100> set print intensity
 */

#include <NimBLEDevice.h>
#include <WiFi.h>
#include <WebServer.h>

// Credentials live in secrets.h (git-ignored). Without it the sketch still builds
// and falls back to its own access point.
#if __has_include("secrets.h")
#include "secrets.h"
#else
#define WIFI_SSID "CHANGE_ME"
#define WIFI_PASS "CHANGE_ME"
#define AP_SSID "MXW01-C3"
#endif

// ---------------------------------------------------------------- protocol ---

static const uint16_t PRINT_W   = 384;
static const uint8_t  ROW_BYTES = PRINT_W / 8;  // 48

static const uint8_t CMD_STATUS      = 0xA1;
static const uint8_t CMD_INTENSITY   = 0xA2;
static const uint8_t CMD_PRINT       = 0xA9;
static const uint8_t CMD_DONE        = 0xAA;
static const uint8_t CMD_BATTERY     = 0xAB;
static const uint8_t CMD_FLUSH       = 0xAD;
static const uint8_t CMD_VERSION     = 0xB1;
static const uint8_t MODE_MONOCHROME = 0x00;

static NimBLEUUID UUID_SVC   ((uint16_t)0xAE30);
static NimBLEUUID UUID_CMD   ((uint16_t)0xAE01);
static NimBLEUUID UUID_NOTIFY((uint16_t)0xAE02);
static NimBLEUUID UUID_DATA  ((uint16_t)0xAE03);

static const char* TARGET_NAME = "MXW01";

// ------------------------------------------------------------------- state ---

static NimBLEClient* cli = nullptr;
static NimBLERemoteCharacteristic* chCmd    = nullptr;
static NimBLERemoteCharacteristic* chNotify = nullptr;
static NimBLERemoteCharacteristic* chData   = nullptr;

static uint8_t intensity = 100;

// The C3 has one 2.4 GHz radio: WiFi traffic and BLE writes fight over the same
// antenna. Turning WiFi off for the duration of a job is what keeps rows from
// failing mid-print; toggle at runtime with the serial command 'c'.
static bool wifiOffDuringPrint = false;

static volatile bool doneFlag = false;      // 0xAA print-complete seen
static volatile int  lastCmd  = -1;         // last frame command byte
static uint8_t       statusBuf[32];
static size_t        statusLen = 0;
static int           batteryPct = -1;
static char          fwVersion[32] = {0};

// --------------------------------------------------------------- crc / frame ---

static uint8_t crc8(const uint8_t* data, size_t n) {
  uint8_t crc = 0;
  for (size_t i = 0; i < n; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return crc;
}

static size_t buildFrame(uint8_t cmd, const uint8_t* payload, uint16_t n, uint8_t* out) {
  out[0] = 0x22; out[1] = 0x21; out[2] = cmd; out[3] = 0x00;
  out[4] = n & 0xFF; out[5] = (n >> 8) & 0xFF;
  if (n) memcpy(out + 6, payload, n);
  out[6 + n] = crc8(payload, n);
  out[7 + n] = 0xFF;
  return 8u + n;
}

static bool sendCmd(uint8_t cmd, const uint8_t* payload = nullptr, uint16_t n = 0) {
  if (!chCmd) return false;
  uint8_t frame[64];
  if (n > sizeof(frame) - 8) return false;
  lastCmd = -1;
  return chCmd->writeValue(frame, buildFrame(cmd, payload, n, frame), false);
}

// ------------------------------------------------------------------ notify ---

static void onNotify(NimBLERemoteCharacteristic* ch, uint8_t* data, size_t len, bool) {
  if (len < 3 || data[0] != 0x22 || data[1] != 0x21) return;
  const uint8_t cmd = data[2];
  lastCmd = cmd;

  if (cmd == CMD_STATUS) {
    statusLen = len < sizeof(statusBuf) ? len : sizeof(statusBuf);
    memcpy(statusBuf, data, statusLen);
  } else if (cmd == CMD_BATTERY && len > 6) {
    batteryPct = data[6];
  } else if (cmd == CMD_VERSION && len > 6) {
    uint16_t l = data[4] | (data[5] << 8);
    if (l > sizeof(fwVersion) - 1) l = sizeof(fwVersion) - 1;
    memcpy(fwVersion, data + 6, l);
    fwVersion[l] = 0;
  } else if (cmd == CMD_DONE) {
    doneFlag = true;
  }
}

static bool waitCmd(uint8_t cmd, uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    if (lastCmd == cmd) return true;
    delay(5);
  }
  return false;
}

// ------------------------------------------------------------------ canvas ---

// Minimal 5x7 font, column-major, bit0 = top row (classic font5x7 layout).
// Covers 0x20..0x5A; unused slots are blank and lowercase maps to uppercase.
static const uint8_t FONT_5X7[59][5] = {
  {0x00,0x00,0x00,0x00,0x00}, // ' '
  {0x00,0x00,0x5F,0x00,0x00}, // !
  {0x00,0x00,0x00,0x00,0x00}, // "
  {0x00,0x00,0x00,0x00,0x00}, // #
  {0x00,0x00,0x00,0x00,0x00}, // $
  {0x00,0x00,0x00,0x00,0x00}, // %
  {0x00,0x00,0x00,0x00,0x00}, // &
  {0x00,0x00,0x00,0x00,0x00}, // '
  {0x00,0x00,0x00,0x00,0x00}, // (
  {0x00,0x00,0x00,0x00,0x00}, // )
  {0x00,0x00,0x00,0x00,0x00}, // *
  {0x00,0x00,0x00,0x00,0x00}, // +
  {0x00,0x00,0x00,0x00,0x00}, // ,
  {0x08,0x08,0x08,0x08,0x08}, // -
  {0x00,0x60,0x60,0x00,0x00}, // .
  {0x20,0x10,0x08,0x04,0x02}, // /
  {0x3E,0x51,0x49,0x45,0x3E}, // 0
  {0x00,0x42,0x7F,0x40,0x00}, // 1
  {0x42,0x61,0x51,0x49,0x46}, // 2
  {0x21,0x41,0x45,0x4B,0x31}, // 3
  {0x18,0x14,0x12,0x7F,0x10}, // 4
  {0x27,0x45,0x45,0x45,0x39}, // 5
  {0x3C,0x4A,0x49,0x49,0x30}, // 6
  {0x01,0x71,0x09,0x05,0x03}, // 7
  {0x36,0x49,0x49,0x49,0x36}, // 8
  {0x06,0x49,0x49,0x29,0x1E}, // 9
  {0x00,0x36,0x36,0x00,0x00}, // :
  {0x00,0x00,0x00,0x00,0x00}, // ;
  {0x00,0x00,0x00,0x00,0x00}, // <
  {0x14,0x14,0x14,0x14,0x14}, // =
  {0x00,0x00,0x00,0x00,0x00}, // >
  {0x00,0x00,0x00,0x00,0x00}, // ?
  {0x00,0x00,0x00,0x00,0x00}, // @
  {0x7E,0x11,0x11,0x11,0x7E}, // A
  {0x7F,0x49,0x49,0x49,0x36}, // B
  {0x3E,0x41,0x41,0x41,0x22}, // C
  {0x7F,0x41,0x41,0x22,0x1C}, // D
  {0x7F,0x49,0x49,0x49,0x41}, // E
  {0x7F,0x09,0x09,0x09,0x01}, // F
  {0x3E,0x41,0x41,0x51,0x32}, // G
  {0x7F,0x08,0x08,0x08,0x7F}, // H
  {0x00,0x41,0x7F,0x41,0x00}, // I
  {0x20,0x40,0x41,0x3F,0x01}, // J
  {0x7F,0x08,0x14,0x22,0x41}, // K
  {0x7F,0x40,0x40,0x40,0x40}, // L
  {0x7F,0x02,0x04,0x02,0x7F}, // M
  {0x7F,0x04,0x08,0x10,0x7F}, // N
  {0x3E,0x41,0x41,0x41,0x3E}, // O
  {0x7F,0x09,0x09,0x09,0x06}, // P
  {0x3E,0x41,0x51,0x21,0x5E}, // Q
  {0x7F,0x09,0x19,0x29,0x46}, // R
  {0x46,0x49,0x49,0x49,0x31}, // S
  {0x01,0x01,0x7F,0x01,0x01}, // T
  {0x3F,0x40,0x40,0x40,0x3F}, // U
  {0x1F,0x20,0x40,0x20,0x1F}, // V
  {0x7F,0x20,0x18,0x20,0x7F}, // W
  {0x63,0x14,0x08,0x14,0x63}, // X
  {0x03,0x04,0x78,0x04,0x03}, // Y
  {0x61,0x51,0x49,0x45,0x43}, // Z
};

static const uint8_t* glyph(char c) {
  if (c >= 'a' && c <= 'z') c = (char)(c - 32);
  if (c < 0x20 || c > 0x5A) return nullptr;
  const uint8_t* g = FONT_5X7[c - 0x20];
  for (int i = 0; i < 5; i++) if (g[i]) return g;
  return (c == ' ') ? g : nullptr;
}

struct Canvas {
  uint8_t* px;
  uint16_t h;

  void clear() { memset(px, 0, (size_t)h * ROW_BYTES); }

  inline void setpx(int x, int y) {
    if ((unsigned)x >= PRINT_W || (unsigned)y >= h) return;
    px[(size_t)y * ROW_BYTES + (x >> 3)] |= (uint8_t)(1u << (x & 7));
  }
  void fill(int x, int y, int w, int hh) {
    for (int j = 0; j < hh; j++)
      for (int i = 0; i < w; i++) setpx(x + i, y + j);
  }
  void hline(int x0, int x1, int y, int t = 1) { fill(x0, y, x1 - x0, t); }

  // Draws one line, returns the advance in pixels.
  int text(int x, int y, const char* s, int scale) {
    int cx = x;
    for (; *s; s++) {
      const uint8_t* g = glyph(*s);
      if (g) {
        for (int col = 0; col < 5; col++)
          for (int row = 0; row < 7; row++)
            if (g[col] & (1u << row))
              fill(cx + col * scale, y + row * scale, scale, scale);
      }
      cx += 6 * scale;
    }
    return cx - x;
  }

  int textWidth(const char* s, int scale) { return (int)strlen(s) * 6 * scale - scale; }

  void textCentered(int y, const char* s, int scale) {
    int w = textWidth(s, scale);
    int x = (PRINT_W - w) / 2;
    text(x < 0 ? 0 : x, y, s, scale);
  }

};

// Height of the last row that contains a black pixel (+1). Free function so the
// Arduino auto-prototype pass never has to know the Canvas type.
static uint16_t trimHeight(const uint8_t* px, uint16_t maxRows) {
  for (int y = maxRows - 1; y >= 0; y--)
    for (int b = 0; b < ROW_BYTES; b++)
      if (px[(size_t)y * ROW_BYTES + b]) return (uint16_t)(y + 1);
  return 0;
}

// ------------------------------------------------------------------ BLE I/O ---

static void dumpGatt() {
  auto svcs = cli->getServices(true);
  if (svcs.empty()) { Serial.println("  (no services)"); return; }
  for (auto* s : svcs) {
    Serial.printf("  svc %s\n", s->getUUID().toString().c_str());
    auto chars = s->getCharacteristics(true);
    for (auto* c : chars)
      Serial.printf("    chr %s  [%s%s]\n", c->getUUID().toString().c_str(),
                    c->canRead() ? "r" : "", c->canWrite() ? "w" : "");
  }
}

static bool resolveCharacteristics() {
  auto* svc = cli->getService(UUID_SVC);
  if (!svc) {
    Serial.println("[!] service AE30 not found, dumping GATT table:");
    dumpGatt();
    return false;
  }
  chCmd    = svc->getCharacteristic(UUID_CMD);
  chNotify = svc->getCharacteristic(UUID_NOTIFY);
  chData   = svc->getCharacteristic(UUID_DATA);
  if (!chCmd || !chNotify || !chData) {
    Serial.println("[!] AE01/AE02/AE03 missing, dumping GATT table:");
    dumpGatt();
    return false;
  }
  if (!chNotify->subscribe(true, onNotify)) {
    Serial.println("[!] notify subscribe failed");
    return false;
  }
  return true;
}

class ScanCB : public NimBLEScanCallbacks {
 public:
  bool verbose = false;
  NimBLEAddress found{};
  bool hit = false;
  int count = 0;

  void onResult(const NimBLEAdvertisedDevice* dev) override {
    count++;
    const bool bySvc = dev->haveServiceUUID() && dev->isAdvertisingService(UUID_SVC);

    String n = dev->haveName() ? String(dev->getName().c_str()) : String();
    String up = n;
    up.toUpperCase();
    const bool byName = up.indexOf(TARGET_NAME) >= 0;

    if (verbose) {
      Serial.printf("   %-18s %-20s rssi %4d  svc:%d name:%d\n",
                    n.length() ? n.c_str() : "(noname)",
                    dev->getAddress().toString().c_str(), dev->getRSSI(),
                    bySvc, byName);
    }
    if (hit || (!byName && !bySvc)) return;

    hit   = true;
    found = dev->getAddress();
    Serial.printf("[+] %s  %s  rssi %d%s\n",
                  n.length() ? n.c_str() : "(noname)",
                  found.toString().c_str(), dev->getRSSI(),
                  byName ? "  (name match)" : "  (service match)");
  }
};

// Arduino's auto-prototype pass hoists function signatures above every type
// definition, so functions must never take ScanCB as a parameter. One global
// instance it is.
static ScanCB gScan;

// One blocking scan round. Returns true when the printer was matched.
static bool scanRound(uint32_t ms, bool verbose) {
  gScan = ScanCB();
  gScan.verbose = verbose;

  auto* scanner = NimBLEDevice::getScan();
  scanner->setScanCallbacks(&gScan, false);
  scanner->setActiveScan(true);
  scanner->setInterval(100);
  scanner->setWindow(80);

  // NimBLE-Arduino 2.x: start() only kicks off the scan and returns false on
  // failure; it does NOT block for the duration. Wait it out before stopping,
  // otherwise the scan lasts 0 ms and no advertiser is ever reported.
  uint32_t t0 = millis();
  if (!scanner->start(ms)) {
    Serial.println("[x] scan could not start (Host Reset / busy)");
    return false;
  }
  while (scanner->isScanning() && millis() - t0 < ms + 1000) delay(20);
  scanner->stop();
  scanner->clearResults();

  if (verbose)
    Serial.printf("[*] scan %.1fs, %d device(s) seen\n", (millis() - t0) / 1000.0, gScan.count);
  return gScan.hit;
}

static void scanDump(uint32_t ms) {
  Serial.printf("[*] dumping every advertiser for %lu ms...\n", (unsigned long)ms);
  scanRound(ms, true);
  if (!gScan.count) Serial.println("[!] no BLE advertisers at all — radio/init problem");
}


static bool connectPrinter(uint32_t scanMs = 8000) {
  if (cli && cli->isConnected()) return true;

  if (!cli) cli = NimBLEDevice::createClient();
  cli->setConnectTimeout(15000);

  bool hit = false;
  for (int round = 1; round <= 2 && !hit; round++) {
    Serial.printf("[*] scan round %d (%lu ms)...\n", round, (unsigned long)scanMs);
    hit = scanRound(scanMs, round == 2);  // dump devices if we are retrying
    if (!hit && round == 2 && !gScan.count)
      Serial.println("[!] no BLE advertisers at all — radio/init problem");
  }

  if (!hit) {
    Serial.println("[x] MXW01 not advertising (powered on? not paired to the phone app?)");
    return false;
  }

  for (int attempt = 1; attempt <= 3; attempt++) {
    Serial.printf("[*] connect attempt %d...\n", attempt);
    if (cli->connect(gScan.found) && cli->isConnected()) {
      Serial.printf("[+] connected, MTU %u\n", cli->getMTU());
      if (resolveCharacteristics()) return true;
      cli->disconnect();
      return false;
    }
    delay(1000 * attempt);
  }
  Serial.println("[x] connect failed");
  return false;
}

static bool ensureConnected() {
  if (cli && cli->isConnected() && chData) return true;
  return connectPrinter();
}

// ---------------------------------------------------------------- printing ---

static bool printRows(const uint8_t* rows, uint16_t count, uint8_t inten) {
  if (!ensureConnected()) return false;
  if (!count) return false;

  Serial.printf("[*] %u lines @ %u%%\n", count, inten);

  uint8_t v = inten;
  if (!sendCmd(CMD_INTENSITY, &v, 1)) { Serial.println("[x] A2 failed"); return false; }
  delay(50);

  uint8_t hdr[4] = {(uint8_t)(count & 0xFF), (uint8_t)(count >> 8), 0x30, MODE_MONOCHROME};
  if (!sendCmd(CMD_PRINT, hdr, 4)) { Serial.println("[x] A9 failed"); return false; }
  delay(50);

  // Pacing has to follow the negotiated connection interval: the controller only
  // has ~16 tx buffers, so blasting 300 rows in a burst overruns them and the
  // write returns false around row 16. One row per interval is the safe rate.
  uint16_t gapMs = 8;
  if (cli && cli->isConnected()) {
    const uint16_t itvl = cli->getConnInfo().getConnInterval();  // 1.25 ms units
    const uint16_t itvlMs = (uint16_t)((itvl * 5 + 3) / 4);
    if (itvlMs > gapMs) gapMs = itvlMs;
    Serial.printf("[*] conn interval %u (%.2f ms) -> %u ms per row\n",
                  itvl, itvl * 1.25f, gapMs);
  }

  uint32_t t0 = millis();
  for (uint16_t y = 0; y < count; y++) {
    bool ok = false;
    for (int attempt = 0; attempt < 4 && !ok; attempt++) {
      ok = chData->writeValue(rows + (size_t)y * ROW_BYTES, ROW_BYTES, false);
      if (!ok) delay(gapMs * 2);  // let the controller drain, then retry
    }
    if (!ok) {
      Serial.printf("[x] row write failed at %u\n", y);
      return false;
    }
    delay(gapMs);
  }
  Serial.printf("[*] %u rows sent in %lu ms\n", count, (unsigned long)(millis() - t0));

  uint8_t z = 0;
  if (!sendCmd(CMD_FLUSH, &z, 1)) { Serial.println("[x] AD failed"); return false; }

  doneFlag = false;
  uint32_t t1 = millis();
  while (millis() - t1 < 90000) {
    if (doneFlag) {
      Serial.printf("[+] print complete (%lu ms)\n", (unsigned long)(millis() - t1));
      return true;
    }
    delay(10);
  }
  Serial.println("[!] no 0xAA within 90 s (paper may still be feeding)");
  return true;
}

static void reportStatus() {
  if (!ensureConnected()) return;
  statusLen = 0;
  uint8_t z = 0;
  if (!sendCmd(CMD_STATUS, &z, 1)) { Serial.println("[x] status send failed"); return; }
  if (!waitCmd(CMD_STATUS, 6000)) { Serial.println("[x] no status answer"); return; }

  Serial.printf("[+] status %s\n", lastCmd == CMD_STATUS ? "ok" : "?");
  if (statusLen >= 14) {
    static const char* STATE[] = {"standby", "printing", "feeding", "ejecting"};
    uint8_t state = statusBuf[6], batt = statusBuf[9];
    uint8_t temp  = statusBuf[10], errFlag = statusBuf[12], err = statusBuf[13];
    batteryPct = batt;
    Serial.printf("    state %s | battery %u%% | temp %u C | error %s\n",
                  state < 4 ? STATE[state] : "?", batt, temp,
                  errFlag == 0 ? "none"
                               : (err == 0x01 || err == 0x09) ? "no paper"
                               : err == 0x04 ? "overheated"
                               : err == 0x08 ? "low battery" : "unknown");
  } else {
    Serial.println("    short status packet");
  }
}

static void reportBattery() {
  if (!ensureConnected()) return;
  batteryPct = -1;
  uint8_t z = 0;
  sendCmd(CMD_BATTERY, &z, 1);
  if (waitCmd(CMD_BATTERY, 6000)) Serial.printf("[+] battery %d%%\n", batteryPct);
  else Serial.println("[x] no battery answer");
}

static void reportVersion() {
  if (!ensureConnected()) return;
  uint8_t z = 0;
  sendCmd(CMD_VERSION, &z, 1);
  if (waitCmd(CMD_VERSION, 6000)) Serial.printf("[+] firmware %s\n", fwVersion);
  else Serial.println("[x] no version answer");
}

// -------------------------------------------------------------- page layout ---

static uint8_t* allocCanvas(uint16_t rows) {
  return (uint8_t*)calloc((size_t)rows * ROW_BYTES, 1);
}

static bool printPage(uint8_t* px, uint16_t maxRows) {
  uint16_t h = trimHeight(px, maxRows);
  if (!h) { Serial.println("[x] empty page"); return false; }
  Serial.printf("[*] page %ux%u\n", PRINT_W, h);
  return printRows(px, h, intensity);
}

static void selfTestPage() {
  const uint16_t MAXR = 320;
  uint8_t* buf = allocCanvas(MAXR);
  if (!buf) { Serial.println("[x] out of memory"); return; }
  Canvas c{buf, MAXR};
  c.clear();

  c.textCentered(10, "MXW01", 5);
  c.textCentered(50, "ESP32-C3", 3);
  c.hline(12, PRINT_W - 12, 80, 2);

  char line[40];
  int y = 94;
  c.text(14, y, "BLE", 2);
  c.text(14 + 6 * 2 * 4, y, "OK", 2);           // right of "BLE "
  y += 24;
  c.text(14, y, "NAME", 2);
  c.text(14 + 6 * 2 * 5, y, TARGET_NAME, 2);    // right of "NAME "
  y += 24;

  if (batteryPct >= 0) {
    snprintf(line, sizeof(line), "BATT %d%%", batteryPct);
  } else {
    snprintf(line, sizeof(line), "BATT ?");
  }
  c.text(14, y, line, 2);
  y += 24;

  if (statusLen >= 14) {
    snprintf(line, sizeof(line), "TEMP %uC", statusBuf[10]);
    c.text(14, y, line, 2);
    y += 24;
  }
  if (fwVersion[0]) {
    c.text(14, y, "FW", 2);
    c.text(14 + 6 * 2 * 3, y, fwVersion, 2);
    y += 24;
  }

  // Head uniformity ramp: 12.5 / 25 / 50 / 75 / 100 % black.
  y += 6;
  c.hline(12, PRINT_W - 12, y, 1);
  y += 6;
  const uint8_t STEP = 48;  // every 48 px, 8 rows
  for (int band = 0; band < 8; band++) {
    for (int yy = 0; yy < 8; yy++) {
      for (int x = 0; x < 360; x++) {
        int mask = (1 << (band + 1)) - 1;  // 1,3,7,...,255
        if (((x * 8 / 45) & 0xFF) < mask) c.setpx(12 + x, y + yy);
      }
    }
    y += 8;
  }
  y += 4;
  c.hline(12, PRINT_W - 12, y, 2);
  y += 10;
  c.textCentered(y, "PRINTED", 2);
  y += 20;
  c.textCentered(y, "BY", 2);
  y += 20;
  c.textCentered(y, "C3", 3);

  printPage(buf, MAXR);
  free(buf);
}

// Renders wrapped 3x text into buf; returns the trimmed page height in rows.
static uint16_t renderText(const char* text, uint8_t* buf, uint16_t maxRows) {
  const int scale = 3;
  const int charW = 6 * scale;
  const int margin = 12;
  const int maxChars = (PRINT_W - 2 * margin) / charW;

  Canvas c{buf, maxRows};
  c.clear();

  int y = 12;
  char line[64];
  int n = 0;
  const char* p = text;

  while (*p && y + 8 * scale < maxRows) {
    // take the next word, wrap on overflow
    while (*p == ' ') p++;
    const char* start = p;
    while (*p && *p != ' ' && *p != '\n') p++;
    int len = (int)(p - start);
    bool nl = (*p == '\n');

    if (n + len + (n ? 1 : 0) > maxChars && n > 0) {
      line[n] = 0;
      c.text(margin, y, line, scale);
      y += 8 * scale + 4;
      n = 0;
    }
    if (n) line[n++] = ' ';
    for (int i = 0; i < len && n < (int)sizeof(line) - 1; i++) line[n++] = start[i];

    if (nl) {
      line[n] = 0;
      c.text(margin, y, line, scale);
      y += 8 * scale + 4;
      n = 0;
      p++;
    }
  }
  if (n) {
    line[n] = 0;
    c.text(margin, y, line, scale);
  }

  return trimHeight(buf, maxRows);
}

static void textPage(const char* text) {
  const uint16_t MAXR = 320;
  uint8_t* buf = allocCanvas(MAXR);
  if (!buf) { Serial.println("[x] out of memory"); return; }
  if (renderText(text, buf, MAXR)) printPage(buf, MAXR);
  else Serial.println("[x] empty page");
  free(buf);
}

// ------------------------------------------------------------------ serial ---

static void handleLine(char* cmd) {
  while (*cmd == ' ') cmd++;
  if (!*cmd) return;

  switch (cmd[0]) {
    case 'p':
      reportStatus();
      selfTestPage();
      break;
    case 't': {
      char* text = cmd + 1;
      while (*text == ' ') text++;
      if (!*text) { Serial.println("[!] usage: t <text>"); break; }
      textPage(text);
      break;
    }
    case 's': reportStatus(); break;
    case 'd': scanDump(12000); break;
    case 'q':
      printWifiQr(WIFI_SSID, WIFI_PASS, "WPA");
      break;
    case 'Q':
      qrDump(WIFI_SSID, WIFI_PASS, "WPA");
      break;
    case 'c':
      wifiOffDuringPrint = !wifiOffDuringPrint;
      Serial.printf("[+] wifi off during print: %s\n", wifiOffDuringPrint ? "on" : "off");
      break;
    case 'w':
      Serial.printf("[*] wifi %s  ip %s  rssi %d\n",
                    WiFi.isConnected() ? "STA" : "AP",
                    WiFi.isConnected() ? WiFi.localIP().toString().c_str()
                                       : WiFi.softAPIP().toString().c_str(),
                    WiFi.isConnected() ? WiFi.RSSI() : 0);
      break;
    case 'b': reportBattery(); break;
    case 'v': reportVersion(); break;
    case 'i': {
      int v = atoi(cmd + 1);
      intensity = (uint8_t)constrain(v, 0, 100);
      Serial.printf("[+] intensity %u%%\n", intensity);
      break;
    }
    default:
      Serial.println("[?] p=selftest t <text> s=status d=scan-dump b=battery v=version i <0-100>");
  }
}



// ---------------------------------------------------------------------- qr ---

// Arduino defines LOW/HIGH as pin levels and qrcodegen has Ecc::LOW / Ecc::HIGH,
// so both macros have to go before the include.
#undef LOW
#undef HIGH
#include "src/qrcodegen.hpp"

// "WIFI:T:WPA;S:<ssid>;P:<pass>;;" — the characters below have to be escaped.
static void qrEscape(const char* in, char* out, size_t cap) {
  size_t o = 0;
  for (const char* p = in; *p && o + 2 < cap; p++) {
    char ch = *p;
    if (ch == '\\' || ch == ';' || ch == ',' || ch == ':' || ch == '"')
      out[o++] = '\\';
    out[o++] = ch;
  }
  out[o] = 0;
}

static void wifiPayload(const char* ssid, const char* pass, const char* auth,
                        char* out, size_t cap) {
  char es[96], ep[96];
  qrEscape(ssid, es, sizeof es);
  qrEscape(pass, ep, sizeof ep);
  snprintf(out, cap, "WIFI:T:%s;S:%s;P:%s;;", auth, es, ep);
}

// Renders the QR sheet; returns a heap buffer of `rowsOut` 48-byte rows.
static uint8_t* buildWifiQrPage(const char* ssid, const char* pass, const char* auth,
                                uint16_t* rowsOut) {
  char payload[256];
  wifiPayload(ssid, pass, auth, payload, sizeof payload);

  qrcodegen::QrCode qr =
      qrcodegen::QrCode::encodeText(payload, qrcodegen::QrCode::Ecc::MEDIUM);
  const int n = qr.getSize();

  const int margin = 4;                                  // px around the symbol
  const int mod = (PRINT_W - 2 * margin) / (n + 8);      // 4-module quiet zone
  if (mod < 2) return nullptr;                           // payload too long

  const int symPx = mod * n;
  const int boxPx = mod * (n + 8);
  const uint16_t MAXR = 520;

  uint8_t* buf = allocCanvas(MAXR);
  if (!buf) return nullptr;
  Canvas c{buf, MAXR};
  c.clear();

  c.textCentered(8, "WIFI", 3);

  const int x0 = (PRINT_W - boxPx) / 2 + 4 * mod;
  const int y0 = 34;
  for (int y = 0; y < n; y++)
    for (int x = 0; x < n; x++)
      if (qr.getModule(x, y)) c.fill(x0 + x * mod, y0 + y * mod, mod, mod);

  int ty = y0 + symPx + 10;
  char line[64];
  snprintf(line, sizeof line, "SSID %s", ssid);
  c.text(8, ty, line, 2);
  ty += 20;
  snprintf(line, sizeof line, "PASS %s", pass);
  c.text(8, ty, line, 2);
  ty += 24;
  c.textCentered(ty, "SCAN TO JOIN", 1);

  *rowsOut = trimHeight(buf, MAXR);
  if (!*rowsOut) { free(buf); return nullptr; }
  return buf;
}

static void printWifiQr(const char* ssid, const char* pass, const char* auth) {
  uint16_t rows = 0;
  uint8_t* buf = buildWifiQrPage(ssid, pass, auth, &rows);
  if (!buf) { Serial.println("[x] qr build failed (payload too long?)"); return; }
  Serial.printf("[*] qr page %u rows\n", rows);
  printRows(buf, rows, intensity);
  free(buf);
}

// Prints the raw module matrix so a host can verify the symbol end to end.
static void qrDump(const char* ssid, const char* pass, const char* auth) {
  char payload[256];
  wifiPayload(ssid, pass, auth, payload, sizeof payload);
  qrcodegen::QrCode qr =
      qrcodegen::QrCode::encodeText(payload, qrcodegen::QrCode::Ecc::MEDIUM);
  const int n = qr.getSize();
  Serial.printf("QRPAYLOAD %s\n", payload);
  Serial.printf("QRSIZE %d\n", n);
  for (int y = 0; y < n; y++) {
    for (int x = 0; x < n; x++) Serial.print(qr.getModule(x, y) ? '1' : '0');
    Serial.println();
  }
}

// ------------------------------------------------------------------- wifi ---

static WebServer server(80);
static bool      wifiUp = false;
static const uint16_t MAX_UPLOAD_ROWS = 800;  // 800 x 48 B = 38 KB

// One queued job; the HTTP handler answers immediately and loop() prints it, so
// the browser does not sit through ~10 s of BLE writes.
static uint8_t*  pendingBuf = nullptr;
static uint16_t  pendingRows = 0;
static uint8_t   pendingIntensity = 100;
static uint32_t  pendingAt = 0;

static const char PAGE_HTML[] PROGMEM = R"HTML(<!doctype html>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>MXW01</title>
<style>
body{font:16px system-ui;margin:18px;max-width:520px}
h2{margin:0 0 14px}
textarea,input,select,button{font:inherit;width:100%;box-sizing:border-box;margin:6px 0;padding:10px;border:1px solid #ccc;border-radius:8px}
button{background:#111;color:#fff;border:0;font-weight:600}
pre{background:#f4f4f4;padding:10px;border-radius:8px;white-space:pre-wrap;font-size:13px}
hr{border:0;border-top:1px solid #ddd;margin:18px 0}
</style>
<h2>MXW01 yazıcı</h2>
<textarea id=t rows=3 placeholder="yazılacak metin"></textarea>
<select id=i><option value=100>100%</option><option value=90>90%</option><option value=80>80%</option><option value=70>70%</option></select>
<button onclick="pt()">Metin yazdır</button>
<hr>
<button onclick="pq()">WiFi QR yazdır</button>
<hr>
<input type=file id=f accept=".pbm,image/x-portable-bitmap">
<button onclick="pp()">PBM yazdır</button>
<hr>
<pre id=s>yükleniyor…</pre>
<script>
const $=x=>document.getElementById(x);
async function st(){try{const r=await fetch('/status');$('s').textContent=await r.text()}catch(e){$('s').textContent='yazıcı yanıt vermiyor'}}
async function pt(){const t=$('t').value.trim();if(!t)return;
 const r=await fetch('/text?i='+$('i').value+'&t='+encodeURIComponent(t));alert(await r.text())}
function b64(buf){const b=new Uint8Array(buf);let s='';const CH=0x8000;
 for(let i=0;i<b.length;i+=CH)s+=String.fromCharCode.apply(null,b.subarray(i,i+CH));
 return btoa(s)}
async function pq(){const r=await fetch('/wifi?i='+$('i').value);alert(await r.text())}
async function pp(){const f=$('f').files[0];if(!f)return;
 const r=await fetch('/pbm?i='+$('i').value,{method:'POST',body:b64(await f.arrayBuffer())});alert(await r.text())}
st();setInterval(st,4000);
</script>)HTML";

// UTF-8 -> the 5x7 font's ASCII range (Turkish letters get folded).
static String toAscii(const String& in) {
  String out;
  out.reserve(in.length());
  size_t i = 0;
  while (i < in.length()) {
    uint8_t c = (uint8_t)in[i];
    if (c < 0x80) { out += (char)c; i++; continue; }

    uint16_t cp = 0x3F;
    size_t step = 1;
    if ((c & 0xE0) == 0xC0 && i + 1 < in.length()) {
      cp = (uint16_t)(((c & 0x1F) << 6) | ((uint8_t)in[i + 1] & 0x3F));
      step = 2;
    } else if ((c & 0xF0) == 0xE0 && i + 2 < in.length()) {
      cp = (uint16_t)(((c & 0x0F) << 12) | (((uint8_t)in[i + 1] & 0x3F) << 6) |
                      ((uint8_t)in[i + 2] & 0x3F));
      step = 3;
    }

    char m = '?';
    switch (cp) {
      case 0x00E7: case 0x00C7: m = 'C'; break;  // c cedilla
      case 0x011F: case 0x011E: m = 'G'; break;  // g breve
      case 0x0131: case 0x0130: m = 'I'; break;  // dotless i
      case 0x00F6: case 0x00D6: m = 'O'; break;  // o umlaut
      case 0x015F: case 0x015E: m = 'S'; break;  // s cedilla
      case 0x00FC: case 0x00DC: m = 'U'; break;  // u umlaut
      default: break;
    }
    out += m;
    i += step;
  }
  return out;
}

static uint8_t argIntensity() {
  if (!server.hasArg("i")) return intensity;
  return (uint8_t)constrain(server.arg("i").toInt(), 0, 100);
}

// WebServer reads any non-form POST body itself and hands it over as the String
// arg "plain" — which truncates at the first 0x00, so binary has to travel
// base64-encoded and gets decoded here.
static int b64val(char c) {
  if (c >= 'A' && c <= 'Z') return c - 'A';
  if (c >= 'a' && c <= 'z') return c - 'a' + 26;
  if (c >= '0' && c <= '9') return c - '0' + 52;
  if (c == '+') return 62;
  if (c == '/') return 63;
  return -1;
}

static size_t b64decode(const char* in, size_t inLen, uint8_t* out, size_t outCap) {
  size_t o = 0;
  uint32_t acc = 0;
  int bits = 0;
  for (size_t i = 0; i < inLen; i++) {
    if (in[i] == '=') break;
    int v = b64val(in[i]);
    if (v < 0) continue;  // skip whitespace / newlines
    acc = (acc << 6) | (uint32_t)v;
    bits += 6;
    if (bits >= 8) {
      bits -= 8;
      if (o < outCap) out[o++] = (uint8_t)((acc >> bits) & 0xFF);
    }
  }
  return o;
}

// Decodes the request body into a fresh buffer. Returns nullptr on failure.
static uint8_t* decodeBody(size_t* outLen) {
  if (!server.hasArg("plain")) return nullptr;
  const String& body = server.arg("plain");
  if (!body.length()) return nullptr;

  size_t cap = body.length() * 3 / 4 + 4;
  uint8_t* buf = (uint8_t*)malloc(cap);
  if (!buf) return nullptr;
  *outLen = b64decode(body.c_str(), body.length(), buf, cap);
  if (!*outLen) { free(buf); return nullptr; }
  return buf;
}

static void queueJob(uint8_t* buf, uint16_t rows, uint8_t inten) {
  if (pendingBuf) free(pendingBuf);
  pendingBuf = buf;
  pendingRows = rows;
  pendingIntensity = inten;
  pendingAt = millis();
}

static void handleRoot() { server.send_P(200, "text/html", PAGE_HTML); }

static void handleStatus() {
  char out[240];
  snprintf(out, sizeof(out),
           "wifi=%s\nip=%s\nrssi=%d\nbattery=%d%%\ntemp=%uC\nfw=%s\nqueue=%u lines\nintensity=%u%%\n",
           wifiUp ? "up" : "down",
           WiFi.isConnected() ? WiFi.localIP().toString().c_str() : WiFi.softAPIP().toString().c_str(),
           WiFi.isConnected() ? WiFi.RSSI() : 0,
           batteryPct,
           statusLen >= 14 ? statusBuf[10] : 0,
           fwVersion[0] ? fwVersion : "?",
           pendingRows, intensity);
  server.send(200, "text/plain", out);
}

static void handleText() {
  if (!server.hasArg("t")) { server.send(400, "text/plain", "missing t"); return; }

  String text = toAscii(server.arg("t"));
  if (!text.length()) { server.send(400, "text/plain", "empty t"); return; }

  const uint16_t MAXR = 320;
  uint8_t* buf = allocCanvas(MAXR);
  if (!buf) { server.send(500, "text/plain", "out of memory"); return; }

  uint16_t h = renderText(text.c_str(), buf, MAXR);
  if (!h) { free(buf); server.send(400, "text/plain", "nothing to print"); return; }

  uint8_t inten = argIntensity();
  queueJob(buf, h, inten);

  char msg[64];
  snprintf(msg, sizeof(msg), "queued %u lines @ %u%%", h, inten);
  server.send(200, "text/plain", msg);
}

// POST /print : base64 of raw 1bpp rows, 48 bytes per row, LSB-first, black = 1.
static void handleRaw() {
  size_t len = 0;
  uint8_t* rows = decodeBody(&len);
  if (!rows) { server.send(400, "text/plain", "body must be base64 rows"); return; }
  if (len % ROW_BYTES) {
    free(rows);
    server.send(400, "text/plain", "decoded body must be N*48 bytes");
    return;
  }
  uint16_t count = (uint16_t)(len / ROW_BYTES);
  if (!count || count > MAX_UPLOAD_ROWS) {
    free(rows);
    server.send(413, "text/plain", "height must be 1..800 rows");
    return;
  }

  uint8_t inten = argIntensity();
  queueJob(rows, count, inten);

  char msg[64];
  snprintf(msg, sizeof(msg), "queued %u lines @ %u%%", count, inten);
  server.send(200, "text/plain", msg);
}

// POST /pbm : binary PBM (P4), any width up to 384, rows MSB-first per byte.
static bool parsePbmHeader(const uint8_t* d, size_t n, uint16_t& w, uint16_t& h,
                           size_t& dataOff) {
  if (n < 8 || d[0] != 'P' || d[1] != '4') return false;
  size_t i = 2;
  long vals[2] = {0, 0};

  for (int k = 0; k < 2; k++) {
    while (i < n) {
      uint8_t c = d[i];
      if (c == '#') { while (i < n && d[i] != '\n') i++; }
      else if (c == ' ' || c == '\t' || c == '\r' || c == '\n') i++;
      else break;
    }
    if (i >= n) return false;

    long v = 0;
    bool any = false;
    while (i < n && d[i] >= '0' && d[i] <= '9') {
      v = v * 10 + (d[i] - '0');
      i++;
      any = true;
      if (v > 100000) return false;
    }
    if (!any) return false;
    vals[k] = v;
  }

  if (i >= n) return false;
  i++;  // single whitespace byte ends the header
  w = (uint16_t)vals[0];
  h = (uint16_t)vals[1];
  dataOff = i;
  return true;
}

static void handlePbm() {
  size_t len = 0;
  uint8_t* raw = decodeBody(&len);
  if (!raw) { server.send(400, "text/plain", "body must be base64 pbm"); return; }

  uint16_t w = 0, h = 0;
  size_t off = 0;
  if (!parsePbmHeader(raw, len, w, h, off)) {
    free(raw);
    server.send(400, "text/plain", "not a P4 PBM (base64 decoded)");
    return;
  }
  if (w < 8 || w > PRINT_W || h < 1 || h > MAX_UPLOAD_ROWS) {
    free(raw);
    server.send(400, "text/plain", "need 8<=width<=384 and 1<=height<=800");
    return;
  }

  const size_t srcRow = (w + 7) / 8;
  if (len < off + srcRow * h) {
    free(raw);
    server.send(400, "text/plain", "truncated PBM data");
    return;
  }

  uint8_t* buf = allocCanvas(h);
  if (!buf) { free(raw); server.send(500, "text/plain", "out of memory"); return; }

  for (uint16_t y = 0; y < h; y++) {
    const uint8_t* src = raw + off + (size_t)y * srcRow;
    uint8_t* dst = buf + (size_t)y * ROW_BYTES;
    for (uint16_t x = 0; x < w; x++) {
      if (src[x >> 3] & (0x80 >> (x & 7))) dst[x >> 3] |= (uint8_t)(1u << (x & 7));
    }
  }
  free(raw);

  uint8_t inten = argIntensity();
  queueJob(buf, h, inten);

  char msg[80];
  snprintf(msg, sizeof(msg), "queued %ux%u pbm @ %u%%", w, h, inten);
  server.send(200, "text/plain", msg);
}

static void handleWifiQr() {
  const String ssid = server.hasArg("s") ? server.arg("s") : String(WIFI_SSID);
  const String pass = server.hasArg("p") ? server.arg("p") : String(WIFI_PASS);
  const String auth = server.hasArg("t") ? server.arg("t") : String("WPA");

  uint16_t rows = 0;
  uint8_t* buf = buildWifiQrPage(ssid.c_str(), pass.c_str(), auth.c_str(), &rows);
  if (!buf) { server.send(400, "text/plain", "qr build failed for that payload"); return; }

  const uint8_t inten = argIntensity();
  queueJob(buf, rows, inten);

  char msg[72];
  snprintf(msg, sizeof msg, "queued wifi qr %u rows @ %u%%", rows, inten);
  server.send(200, "text/plain", msg);
}

static void startWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[*] WiFi '%s' ...\n", WIFI_SSID);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) delay(200);

  if (WiFi.status() == WL_CONNECTED) {
    wifiUp = true;
    Serial.printf("[+] http://%s/  (rssi %d)\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
    return;
  }

  Serial.println("[!] WiFi join failed, starting fallback AP");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID);
  wifiUp = true;
  Serial.printf("[+] AP '%s' -> http://%s/\n",
                AP_SSID, WiFi.softAPIP().toString().c_str());
}

static void startHttp() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/text", HTTP_GET, handleText);
  server.on("/print", HTTP_POST, handleRaw);
  server.on("/pbm", HTTP_POST, handlePbm);
  server.on("/wifi", HTTP_GET, handleWifiQr);
  server.onNotFound([]() { server.send(404, "text/plain", "not found"); });
  server.begin();
  Serial.println("[*] http server on port 80");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nMXW01 thermal printer / ESP32-C3");
  Serial.printf("host uptime %lu ms\n", (unsigned long)millis());

  startWifi();
  startHttp();

  NimBLEDevice::init("");
  NimBLEDevice::setMTU(247);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  if (connectPrinter()) {
    reportStatus();
    reportVersion();
  }

  Serial.println("[?] p=selftest t <text> s=status d=scan-dump w=wifi c=coex q=wifi-qr Q=qr-dump b=battery v=version i <0-100>");
}

void loop() {
  static char buf[96];
  static size_t n = 0;

  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      buf[n] = 0;
      if (n) handleLine(buf);
      n = 0;
      continue;
    }
    if (n < sizeof(buf) - 1) buf[n++] = ch;
  }

  if (wifiUp) server.handleClient();

  // Print queued HTTP jobs here, outside the request handler: the browser gets
  // its answer immediately instead of waiting for the whole BLE transfer.
  if (pendingBuf && millis() - pendingAt > 400) {
    uint8_t* b = pendingBuf;
    uint16_t rows = pendingRows;
    uint8_t inten = pendingIntensity;
    pendingBuf = nullptr;
    pendingRows = 0;

    Serial.printf("[*] http job: %u lines @ %u%%\n", rows, inten);

    const bool radioHandover = wifiOffDuringPrint && wifiUp;
    if (radioHandover) {
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      delay(150);
      Serial.println("[*] wifi off for print");
    }

    if (!printRows(b, rows, inten))
      Serial.println("[x] http job failed");
    free(b);

    if (radioHandover) {
      startWifi();
      server.stop();
      server.begin();
    }
  }

  delay(10);
}
