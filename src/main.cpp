#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <RadioLib.h>
#include <mbedtls/md.h>
#include <esp_system.h>

// Holden LoRa authenticated repeater
// Target: Seeed Studio XIAO ESP32-C3 + DX-LR20 (LLCC68) adapter
//
// This is a half-duplex store-and-forward repeater. It receives a LoRa frame,
// verifies a 128-bit truncated HMAC-SHA256 tag using a 256-bit network key,
// rejects stale/replayed counters, decrements TTL, re-authenticates the frame,
// then retransmits it using the same radio.

static constexpr uint8_t FRAME_MAGIC0 = 'H';
static constexpr uint8_t FRAME_MAGIC1 = 'L';
static constexpr uint8_t FRAME_VERSION = 1;
static constexpr size_t FRAME_HEADER_LEN = 19;
static constexpr size_t FRAME_TAG_LEN = 16;
static constexpr size_t FRAME_MAX_LEN = 240;

// Header offsets
static constexpr size_t OFF_MAGIC0 = 0;
static constexpr size_t OFF_MAGIC1 = 1;
static constexpr size_t OFF_VERSION = 2;
static constexpr size_t OFF_NETWORK = 3;   // 2 bytes, BE
static constexpr size_t OFF_SENDER = 5;    // 4 bytes, BE
static constexpr size_t OFF_COUNTER = 9;   // 8 bytes, BE
static constexpr size_t OFF_TTL = 17;
static constexpr size_t OFF_PAYLOAD_LEN = 18;

struct Config {
  bool configured = false;
  float frequency = 915.000f;
  float bandwidth = 125.0f;
  uint8_t spreadingFactor = 9;
  uint8_t codingRate = 7;
  uint8_t syncWord = 0x12;
  int8_t txPower = 22;
  uint16_t preamble = 12;
  uint16_t networkId = 0x484C;

  // XIAO ESP32-C3 defaults. D8/D9/D10 are the board's documented SPI pins.
  int pinSck = 8;      // D8 / GPIO8
  int pinMiso = 9;     // D9 / GPIO9
  int pinMosi = 10;    // D10 / GPIO10
  int pinNss = 5;      // D3 / GPIO5
  int pinDio1 = 4;     // D2 / GPIO4
  int pinNrst = 3;     // D1 / GPIO3
  int pinBusy = 21;    // D6 / GPIO21
  int pinRxEn = 6;     // D4 / GPIO6
  int pinTxEn = 7;     // D5 / GPIO7

  uint8_t key[32] = {0};
};

Config cfg;
Preferences prefs;
WebServer server(80);
Module* radioModule = nullptr;
LLCC68* radio = nullptr;
volatile bool receivedFlag = false;
bool radioOk = false;

uint32_t rxCount = 0;
uint32_t forwardedCount = 0;
uint32_t authRejectCount = 0;
uint32_t replayRejectCount = 0;
uint32_t malformedCount = 0;
float lastRssi = 0;
float lastSnr = 0;
String lastReason = "boot";
String apSsid;

static uint16_t readU16BE(const uint8_t* p) {
  return (uint16_t(p[0]) << 8) | p[1];
}
static uint32_t readU32BE(const uint8_t* p) {
  return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | p[3];
}
static uint64_t readU64BE(const uint8_t* p) {
  uint64_t v = 0;
  for (int i = 0; i < 8; ++i) v = (v << 8) | p[i];
  return v;
}

static String hexByte(uint8_t b) {
  const char* h = "0123456789ABCDEF";
  String s;
  s += h[(b >> 4) & 0xF];
  s += h[b & 0xF];
  return s;
}

static String keyToHex() {
  String out;
  out.reserve(64);
  for (uint8_t b : cfg.key) out += hexByte(b);
  return out;
}

static bool parseHexKey(String s, uint8_t out[32]) {
  s.trim();
  s.replace(" ", "");
  s.replace(":", "");
  if (s.length() != 64) return false;
  auto nib = [](char c)->int {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
  };
  for (int i = 0; i < 32; ++i) {
    int hi = nib(s[i * 2]);
    int lo = nib(s[i * 2 + 1]);
    if (hi < 0 || lo < 0) return false;
    out[i] = uint8_t((hi << 4) | lo);
  }
  return true;
}

static void generateKey(uint8_t out[32]) {
  for (int i = 0; i < 32; i += 4) {
    uint32_t r = esp_random();
    memcpy(out + i, &r, 4);
  }
}

static bool hmac16(const uint8_t* data, size_t len, uint8_t out[FRAME_TAG_LEN]) {
  const mbedtls_md_info_t* md = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  if (!md) return false;
  uint8_t digest[32];
  int rc = mbedtls_md_hmac(md, cfg.key, sizeof(cfg.key), data, len, digest);
  if (rc != 0) return false;
  memcpy(out, digest, FRAME_TAG_LEN);
  return true;
}

static bool constantTimeEqual(const uint8_t* a, const uint8_t* b, size_t n) {
  uint8_t diff = 0;
  for (size_t i = 0; i < n; ++i) diff |= a[i] ^ b[i];
  return diff == 0;
}

static String counterKey(uint32_t sender) {
  char buf[13];
  snprintf(buf, sizeof(buf), "c%08lX", (unsigned long)sender);
  return String(buf);
}

static uint64_t lastCounterFor(uint32_t sender) {
  String k = counterKey(sender);
  return prefs.getULong64(k.c_str(), 0);
}

static void rememberCounter(uint32_t sender, uint64_t value) {
  String k = counterKey(sender);
  prefs.putULong64(k.c_str(), value);
}

static void saveConfig() {
  prefs.putBool("configured", cfg.configured);
  prefs.putFloat("freq", cfg.frequency);
  prefs.putFloat("bw", cfg.bandwidth);
  prefs.putUChar("sf", cfg.spreadingFactor);
  prefs.putUChar("cr", cfg.codingRate);
  prefs.putUChar("sync", cfg.syncWord);
  prefs.putChar("pwr", cfg.txPower);
  prefs.putUShort("pre", cfg.preamble);
  prefs.putUShort("net", cfg.networkId);
  prefs.putInt("sck", cfg.pinSck);
  prefs.putInt("miso", cfg.pinMiso);
  prefs.putInt("mosi", cfg.pinMosi);
  prefs.putInt("nss", cfg.pinNss);
  prefs.putInt("dio1", cfg.pinDio1);
  prefs.putInt("nrst", cfg.pinNrst);
  prefs.putInt("busy", cfg.pinBusy);
  prefs.putInt("rxen", cfg.pinRxEn);
  prefs.putInt("txen", cfg.pinTxEn);
  prefs.putBytes("key", cfg.key, sizeof(cfg.key));
}

static void loadConfig() {
  cfg.configured = prefs.getBool("configured", false);
  cfg.frequency = prefs.getFloat("freq", cfg.frequency);
  cfg.bandwidth = prefs.getFloat("bw", cfg.bandwidth);
  cfg.spreadingFactor = prefs.getUChar("sf", cfg.spreadingFactor);
  cfg.codingRate = prefs.getUChar("cr", cfg.codingRate);
  cfg.syncWord = prefs.getUChar("sync", cfg.syncWord);
  cfg.txPower = prefs.getChar("pwr", cfg.txPower);
  cfg.preamble = prefs.getUShort("pre", cfg.preamble);
  cfg.networkId = prefs.getUShort("net", cfg.networkId);
  cfg.pinSck = prefs.getInt("sck", cfg.pinSck);
  cfg.pinMiso = prefs.getInt("miso", cfg.pinMiso);
  cfg.pinMosi = prefs.getInt("mosi", cfg.pinMosi);
  cfg.pinNss = prefs.getInt("nss", cfg.pinNss);
  cfg.pinDio1 = prefs.getInt("dio1", cfg.pinDio1);
  cfg.pinNrst = prefs.getInt("nrst", cfg.pinNrst);
  cfg.pinBusy = prefs.getInt("busy", cfg.pinBusy);
  cfg.pinRxEn = prefs.getInt("rxen", cfg.pinRxEn);
  cfg.pinTxEn = prefs.getInt("txen", cfg.pinTxEn);

  if (prefs.getBytesLength("key") == sizeof(cfg.key)) {
    prefs.getBytes("key", cfg.key, sizeof(cfg.key));
  } else {
    generateKey(cfg.key);
    prefs.putBytes("key", cfg.key, sizeof(cfg.key));
  }
}

void IRAM_ATTR onPacketReceived() {
  receivedFlag = true;
}

static void destroyRadio() {
  radioOk = false;
  if (radio) {
    delete radio;
    radio = nullptr;
  }
  if (radioModule) {
    delete radioModule;
    radioModule = nullptr;
  }
  SPI.end();
}

static bool startRadio() {
  destroyRadio();

  Serial.printf("Radio pins: SCK=%d MISO=%d MOSI=%d NSS=%d DIO1=%d RST=%d BUSY=%d RXEN=%d TXEN=%d\n",
                cfg.pinSck, cfg.pinMiso, cfg.pinMosi, cfg.pinNss, cfg.pinDio1,
                cfg.pinNrst, cfg.pinBusy, cfg.pinRxEn, cfg.pinTxEn);

  SPI.begin(cfg.pinSck, cfg.pinMiso, cfg.pinMosi, cfg.pinNss);
  static SPISettings radioSpiSettings(4000000, MSBFIRST, SPI_MODE0);
  radioModule = new Module(cfg.pinNss, cfg.pinDio1, cfg.pinNrst, cfg.pinBusy, SPI, radioSpiSettings);
  radio = new LLCC68(radioModule);
  radio->setRfSwitchPins(cfg.pinRxEn, cfg.pinTxEn);

  // DX-LR20 uses a crystal (tcxoVoltage = 0). Private sync word 0x12 by default.
  int16_t state = radio->begin(cfg.frequency, cfg.bandwidth, cfg.spreadingFactor,
                               cfg.codingRate, cfg.syncWord, cfg.txPower,
                               cfg.preamble, 0.0, false);
  if (state != RADIOLIB_ERR_NONE) {
    lastReason = "radio init failed: " + String(state);
    Serial.println(lastReason);
    return false;
  }

  radio->setPacketReceivedAction(onPacketReceived);
  state = radio->startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    lastReason = "startReceive failed: " + String(state);
    Serial.println(lastReason);
    return false;
  }

  lastReason = "radio ready";
  radioOk = true;
  Serial.println("DX-LR20 radio ready");
  return true;
}

static bool validateAndPrepareRelay(uint8_t* frame, size_t len) {
  if (len < FRAME_HEADER_LEN + FRAME_TAG_LEN || len > FRAME_MAX_LEN) {
    malformedCount++;
    lastReason = "bad length";
    return false;
  }
  if (frame[OFF_MAGIC0] != FRAME_MAGIC0 || frame[OFF_MAGIC1] != FRAME_MAGIC1 || frame[OFF_VERSION] != FRAME_VERSION) {
    malformedCount++;
    lastReason = "wrong magic/version";
    return false;
  }
  if (readU16BE(frame + OFF_NETWORK) != cfg.networkId) {
    authRejectCount++;
    lastReason = "wrong network";
    return false;
  }

  const uint8_t payloadLen = frame[OFF_PAYLOAD_LEN];
  const size_t signedLen = FRAME_HEADER_LEN + payloadLen;
  const size_t expectedLen = signedLen + FRAME_TAG_LEN;
  if (expectedLen != len) {
    malformedCount++;
    lastReason = "payload length mismatch";
    return false;
  }

  uint8_t expectedTag[FRAME_TAG_LEN];
  if (!hmac16(frame, signedLen, expectedTag) || !constantTimeEqual(expectedTag, frame + signedLen, FRAME_TAG_LEN)) {
    authRejectCount++;
    lastReason = "bad HMAC";
    return false;
  }

  const uint32_t sender = readU32BE(frame + OFF_SENDER);
  const uint64_t counter = readU64BE(frame + OFF_COUNTER);
  const uint64_t last = lastCounterFor(sender);
  if (counter <= last) {
    replayRejectCount++;
    lastReason = "replay/stale counter";
    return false;
  }

  if (frame[OFF_TTL] == 0) {
    malformedCount++;
    lastReason = "TTL zero";
    return false;
  }

  // Remember before TX. This prevents a reflected copy of the same valid packet from
  // triggering repeated retransmits if the radio hears its own forwarded frame.
  rememberCounter(sender, counter);

  frame[OFF_TTL]--;
  if (!hmac16(frame, signedLen, frame + signedLen)) {
    authRejectCount++;
    lastReason = "HMAC recompute failed";
    return false;
  }
  return true;
}

static void processRadioPacket() {
  receivedFlag = false;
  if (!radio || !radioOk) return;

  size_t len = radio->getPacketLength();
  if (len == 0 || len > FRAME_MAX_LEN) {
    malformedCount++;
    lastReason = "invalid radio packet length";
    radio->startReceive();
    return;
  }

  uint8_t frame[FRAME_MAX_LEN];
  int16_t state = radio->readData(frame, len);
  if (state != RADIOLIB_ERR_NONE) {
    lastReason = "readData failed: " + String(state);
    radio->startReceive();
    return;
  }

  rxCount++;
  lastRssi = radio->getRSSI();
  lastSnr = radio->getSNR();

  if (validateAndPrepareRelay(frame, len)) {
    // Small jitter keeps two repeaters from always colliding if you add another later.
    delay(40 + (esp_random() % 120));
    state = radio->transmit(frame, len);
    if (state == RADIOLIB_ERR_NONE) {
      forwardedCount++;
      lastReason = "forwarded";
    } else {
      lastReason = "TX failed: " + String(state);
    }
  }

  state = radio->startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    radioOk = false;
    lastReason = "RX restart failed: " + String(state);
  }
}

static String htmlEscape(String s) {
  s.replace("&", "&amp;");
  s.replace("<", "&lt;");
  s.replace(">", "&gt;");
  s.replace("\"", "&quot;");
  return s;
}

static String pageHtml() {
  String h;
  h.reserve(12000);
  h += F("<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>");
  h += F("<title>Holden LoRa Repeater</title><style>");
  h += F("body{font-family:system-ui,-apple-system,Segoe UI,sans-serif;background:#0b1020;color:#edf2ff;margin:0;padding:20px}.wrap{max-width:900px;margin:auto}.card{background:#151c31;border:1px solid #2b3658;border-radius:16px;padding:18px;margin:14px 0}h1,h2{margin-top:0}label{display:block;margin:10px 0 4px;color:#b9c7ef}input{width:100%;box-sizing:border-box;padding:10px;border-radius:9px;border:1px solid #41507a;background:#0e1529;color:white}button{background:#6d7dff;color:white;border:0;border-radius:10px;padding:11px 15px;font-weight:700;cursor:pointer;margin-top:12px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:10px}.stat{font-size:1.7rem;font-weight:800}.muted{color:#a8b3d2}.ok{color:#7bf0aa}.bad{color:#ff8a9a}code{background:#0a1123;padding:2px 5px;border-radius:5px}.keyrow{display:flex;gap:8px}.keyrow input{flex:1}.keyrow button{margin:0;white-space:nowrap}small{color:#9eacd0}</style></head><body><div class='wrap'>");
  h += F("<h1>Holden LoRa Repeater</h1><div class='card'><h2>Status</h2>");
  h += F("<p>Radio: <b class='"); h += radioOk ? "ok'>READY" : "bad'>NOT READY"; h += F("</b></p>");
  h += "<p class='muted'>" + htmlEscape(lastReason) + "</p>";
  h += F("<div class='grid'>");
  h += "<div><div class='stat'>" + String(rxCount) + "</div><small>received</small></div>";
  h += "<div><div class='stat'>" + String(forwardedCount) + "</div><small>forwarded</small></div>";
  h += "<div><div class='stat'>" + String(authRejectCount) + "</div><small>auth/network rejects</small></div>";
  h += "<div><div class='stat'>" + String(replayRejectCount) + "</div><small>replay rejects</small></div>";
  h += F("</div><p>Last RSSI/SNR: <code>"); h += String(lastRssi,1); h += " dBm / "; h += String(lastSnr,1); h += F(" dB</code></p></div>");

  h += F("<div class='card'><h2>Network + security</h2><form method='post' action='/save'>");
  h += F("<label>256-bit network key (64 hex characters)</label><div class='keyrow'><input id='key' name='key' autocomplete='off' value='"); h += keyToHex(); h += F("'><button type='button' onclick='genKey()'>Generate</button></div>");
  h += F("<label>Network ID</label><input name='network' value='"); h += String(cfg.networkId); h += F("'>");

  h += F("<h2 style='margin-top:22px'>LoRa PHY</h2><div class='grid'>");
  h += F("<div><label>Frequency MHz</label><input name='freq' value='"); h += String(cfg.frequency,3); h += F("'></div>");
  h += F("<div><label>Bandwidth kHz</label><input name='bw' value='"); h += String(cfg.bandwidth,1); h += F("'></div>");
  h += F("<div><label>Spreading factor</label><input name='sf' value='"); h += String(cfg.spreadingFactor); h += F("'></div>");
  h += F("<div><label>Coding rate denominator</label><input name='cr' value='"); h += String(cfg.codingRate); h += F("'></div>");
  h += F("<div><label>Sync word (decimal)</label><input name='sync' value='"); h += String(cfg.syncWord); h += F("'></div>");
  h += F("<div><label>TX power dBm</label><input name='pwr' value='"); h += String(cfg.txPower); h += F("'></div>");
  h += F("</div>");

  h += F("<h2 style='margin-top:22px'>DX-LR20 pins (GPIO numbers)</h2><div class='grid'>");
  auto pinField = [&](const char* label, const char* name, int value) {
    h += "<div><label>"; h += label; h += "</label><input name='"; h += name; h += "' value='"; h += String(value); h += "'></div>";
  };
  pinField("SCK", "sck", cfg.pinSck); pinField("MISO", "miso", cfg.pinMiso); pinField("MOSI", "mosi", cfg.pinMosi);
  pinField("NSS/CS", "nss", cfg.pinNss); pinField("DIO1", "dio1", cfg.pinDio1); pinField("NRST", "nrst", cfg.pinNrst);
  pinField("BUSY", "busy", cfg.pinBusy); pinField("RXEN", "rxen", cfg.pinRxEn); pinField("TXEN", "txen", cfg.pinTxEn);
  h += F("</div><button type='submit'>Save and reboot</button></form></div>");

  h += F("<div class='card'><h2>Setup notes</h2><p>This repeater only forwards frames that match the network ID, have a valid HMAC tag, and use a counter newer than the last one seen from that sender.</p><p class='muted'>The endpoint firmware at your house and your girlfriend's Heltec V3 must use the same key, network ID, frequency, bandwidth, spreading factor, coding rate, sync word, and packet format.</p><form method='post' action='/reset' onsubmit=\"return confirm('Erase repeater settings and replay counters?')\"><button>Factory-reset settings</button></form></div>");

  h += F("<script>function genKey(){const a=new Uint8Array(32);crypto.getRandomValues(a);document.getElementById('key').value=[...a].map(x=>x.toString(16).padStart(2,'0')).join('').toUpperCase()}</script>");
  h += F("</div></body></html>");
  return h;
}

static long argLong(const char* name, long fallback) {
  if (!server.hasArg(name)) return fallback;
  return strtol(server.arg(name).c_str(), nullptr, 0);
}

static float argFloat(const char* name, float fallback) {
  if (!server.hasArg(name)) return fallback;
  return server.arg(name).toFloat();
}

static void handleSave() {
  uint8_t newKey[32];
  if (!server.hasArg("key") || !parseHexKey(server.arg("key"), newKey)) {
    server.send(400, "text/plain", "Key must be exactly 64 hexadecimal characters.");
    return;
  }
  memcpy(cfg.key, newKey, 32);
  cfg.networkId = uint16_t(argLong("network", cfg.networkId));
  cfg.frequency = argFloat("freq", cfg.frequency);
  cfg.bandwidth = argFloat("bw", cfg.bandwidth);
  cfg.spreadingFactor = uint8_t(argLong("sf", cfg.spreadingFactor));
  cfg.codingRate = uint8_t(argLong("cr", cfg.codingRate));
  cfg.syncWord = uint8_t(argLong("sync", cfg.syncWord));
  cfg.txPower = int8_t(argLong("pwr", cfg.txPower));
  cfg.pinSck = argLong("sck", cfg.pinSck);
  cfg.pinMiso = argLong("miso", cfg.pinMiso);
  cfg.pinMosi = argLong("mosi", cfg.pinMosi);
  cfg.pinNss = argLong("nss", cfg.pinNss);
  cfg.pinDio1 = argLong("dio1", cfg.pinDio1);
  cfg.pinNrst = argLong("nrst", cfg.pinNrst);
  cfg.pinBusy = argLong("busy", cfg.pinBusy);
  cfg.pinRxEn = argLong("rxen", cfg.pinRxEn);
  cfg.pinTxEn = argLong("txen", cfg.pinTxEn);
  cfg.configured = true;
  saveConfig();
  server.send(200, "text/html", "<h2>Saved.</h2><p>Repeater is rebooting...</p>");
  delay(400);
  ESP.restart();
}

static void handleReset() {
  prefs.clear();
  server.send(200, "text/html", "<h2>Settings erased.</h2><p>Rebooting...</p>");
  delay(400);
  ESP.restart();
}

static void startWeb() {
  uint64_t mac = ESP.getEfuseMac();
  char suffix[7];
  snprintf(suffix, sizeof(suffix), "%06lX", (unsigned long)(mac & 0xFFFFFF));
  apSsid = "HOLDEN-LORA-" + String(suffix);

  WiFi.mode(WIFI_AP);
  // Setup password is intentionally separate from the LoRa network key.
  WiFi.softAP(apSsid.c_str(), "repeater-setup");
  Serial.print("Setup AP: "); Serial.println(apSsid);
  Serial.print("Open: http://"); Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, [](){ server.send(200, "text/html", pageHtml()); });
  server.on("/save", HTTP_POST, handleSave);
  server.on("/reset", HTTP_POST, handleReset);
  server.onNotFound([](){ server.sendHeader("Location", "/"); server.send(302, "text/plain", ""); });
  server.begin();
}

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println("\nHolden LoRa Repeater v0.1.0");
  prefs.begin("holden-lora", false);
  loadConfig();
  startWeb();
  startRadio();
}

void loop() {
  server.handleClient();
  if (receivedFlag) processRadioPacket();

  static uint32_t lastRetry = 0;
  if (!radioOk && millis() - lastRetry > 5000) {
    lastRetry = millis();
    startRadio();
  }
  delay(2);
}
