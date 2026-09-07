#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <RadioLib.h>
#include <mbedtls/md.h>
#include <esp_system.h>

static constexpr uint32_t UART_BAUD = 230400;
static constexpr uint8_t LINK_MAGIC0 = 0xA5;
static constexpr uint8_t LINK_MAGIC1 = 0x5A;
static constexpr uint8_t LINK_DATA = 0x01;
static constexpr uint8_t LINK_TX_BEGIN = 0x02;
static constexpr uint8_t LINK_TX_END = 0x03;
static constexpr uint8_t LINK_HEARTBEAT = 0x04;
static constexpr size_t HOLDEN_LINK_MAX = 220;
static constexpr size_t HDR = 19;
static constexpr size_t TAG = 16;
static constexpr size_t MAX_FRAME = 240;
static constexpr size_t MAX_BRIDGED_PAYLOAD = MAX_FRAME - HDR - TAG;

#ifdef HOLDEN_TARGET_S2
static const char *BOARD_NAME = "Adafruit ESP32-S2 Feather";
static const char *BOARD_TAG = "S2";
static const int UART_RX_PIN = RX;
static const int UART_TX_PIN = TX;
static const int CC_CSN = 10;
static const int CC_GDO0 = 5;
static const int CC_GDO2 = 6;
struct Config {
  float freq = 433.92f;
  float br = 4.8f;
  float dev = 5.0f;
  float bw = 58.0f;
  int csn = CC_CSN;
  int gdo0 = CC_GDO0;
  int gdo2 = CC_GDO2;
} cfg;
#else
static const char *BOARD_NAME = "Seeed XIAO ESP32-C3";
static const char *BOARD_TAG = "C3";
static const int UART_TX_PIN = D6;
static const int UART_RX_PIN = D7;
static const int DEF_SCK = D8;
static const int DEF_MISO = D9;
static const int DEF_MOSI = D10;
static const int DEF_NSS = D0;
static const int DEF_RST = D1;
static const int DEF_DIO1 = D2;
static const int DEF_BUSY = D3;
static const int DEF_TXEN = D4;
static const int DEF_RXEN = D5;
struct Config {
  float freq = 915.0f;
  float bw = 125.0f;
  uint8_t sf = 9;
  uint8_t cr = 7;
  uint8_t sync = 0x12;
  int8_t power = 22;
  uint16_t preamble = 12;
  uint16_t network = 0x484C;
  int sck = DEF_SCK;
  int miso = DEF_MISO;
  int mosi = DEF_MOSI;
  int nss = DEF_NSS;
  int dio1 = DEF_DIO1;
  int rst = DEF_RST;
  int busy = DEF_BUSY;
  int rxen = DEF_RXEN;
  int txen = DEF_TXEN;
  uint8_t key[32]{};
} cfg;
#endif

Preferences prefs;
WebServer server(80);
HardwareSerial LinkSerial(1);
bool apOk = false;
bool radioOk = false;
String ssid;
String statusText = "boot";
uint32_t dropCount = 0;
uint32_t linkLastRx = 0;
uint32_t linkLastHeartbeatTx = 0;

#ifdef HOLDEN_TARGET_S2
Module *ccModule = nullptr;
CC1101 *ccRadio = nullptr;
volatile bool packetReady = false;
bool ccMuted = false;
uint32_t ccMutedSince = 0;
uint32_t ccResumeAt = 0;
uint32_t ccRxCount = 0;
uint32_t uartSentCount = 0;
float lastRssi = 0;
#else
Module *dxModule = nullptr;
LLCC68 *dxRadio = nullptr;
uint32_t uartRxCount = 0;
uint32_t txCount = 0;
int16_t lastTxResult = 0;
#endif

static bool linkConnected() {
  return linkLastRx != 0 && (uint32_t)(millis() - linkLastRx) < 3000;
}

static uint16_t crc16Update(uint16_t crc, uint8_t b) {
  crc ^= (uint16_t)b << 8;
  for (uint8_t i = 0; i < 8; ++i) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  return crc;
}

static uint16_t linkCrc(uint8_t type, uint16_t len, const uint8_t *data) {
  uint16_t crc = 0xFFFF;
  crc = crc16Update(crc, type);
  crc = crc16Update(crc, (uint8_t)(len & 0xFF));
  crc = crc16Update(crc, (uint8_t)(len >> 8));
  if (data) for (uint16_t i = 0; i < len; ++i) crc = crc16Update(crc, data[i]);
  return crc;
}

static void sendLink(uint8_t type, const uint8_t *data = nullptr, uint16_t len = 0) {
  uint16_t crc = linkCrc(type, len, data);
  LinkSerial.write(LINK_MAGIC0); LinkSerial.write(LINK_MAGIC1); LinkSerial.write(type);
  LinkSerial.write((uint8_t)(len & 0xFF)); LinkSerial.write((uint8_t)(len >> 8));
  if (len && data) LinkSerial.write(data, len);
  LinkSerial.write((uint8_t)(crc & 0xFF)); LinkSerial.write((uint8_t)(crc >> 8));
  LinkSerial.flush();
}

static void serviceHeartbeat() {
  uint32_t now = millis();
  if ((uint32_t)(now - linkLastHeartbeatTx) >= 1000) {
    linkLastHeartbeatTx = now;
    sendLink(LINK_HEARTBEAT);
  }
}

static long argI(const char *name, long d) {
  return server.hasArg(name) ? strtol(server.arg(name).c_str(), nullptr, 0) : d;
}
static float argF(const char *name, float d) {
  return server.hasArg(name) ? server.arg(name).toFloat() : d;
}

#ifndef HOLDEN_TARGET_S2
static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }
static void put32(uint8_t *p, uint32_t v) { p[0]=(uint8_t)(v>>24); p[1]=(uint8_t)(v>>16); p[2]=(uint8_t)(v>>8); p[3]=(uint8_t)v; }
static void put64(uint8_t *p, uint64_t v) { for (int i = 7; i >= 0; --i) { p[i] = (uint8_t)v; v >>= 8; } }
static void newKey() {
  for (int i = 0; i < 32; i += 4) { uint32_t r = esp_random(); memcpy(cfg.key + i, &r, 4); }
}
static String keyHex() {
  static const char *hex = "0123456789ABCDEF";
  String out; out.reserve(64);
  for (uint8_t b : cfg.key) { out += hex[b >> 4]; out += hex[b & 0x0F]; }
  return out;
}
static bool parseKey(String s) {
  s.trim(); s.replace(" ", ""); s.replace(":", ""); if (s.length() != 64) return false;
  auto nib = [](char c)->int { if(c>='0'&&c<='9')return c-'0'; if(c>='a'&&c<='f')return c-'a'+10; if(c>='A'&&c<='F')return c-'A'+10; return -1; };
  for (int i=0;i<32;i++) { int a=nib(s[i*2]), b=nib(s[i*2+1]); if(a<0||b<0)return false; cfg.key[i]=(uint8_t)((a<<4)|b); }
  return true;
}
static bool mac16(const uint8_t *data, size_t len, uint8_t *out) {
  const mbedtls_md_info_t *md = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256); if (!md) return false;
  uint8_t full[32]; if (mbedtls_md_hmac(md, cfg.key, 32, data, len, full) != 0) return false; memcpy(out, full, TAG); return true;
}
static uint32_t repeaterId() { uint64_t m = ESP.getEfuseMac(); return (uint32_t)(m ^ (m >> 32)); }
#endif

static void saveConfig() {
#ifdef HOLDEN_TARGET_S2
  prefs.putFloat("ccf", cfg.freq); prefs.putFloat("ccbr", cfg.br); prefs.putFloat("ccdev", cfg.dev); prefs.putFloat("ccbw", cfg.bw);
  prefs.putInt("cccs", cfg.csn); prefs.putInt("ccg0", cfg.gdo0); prefs.putInt("ccg2", cfg.gdo2);
#else
  prefs.putFloat("f",cfg.freq); prefs.putFloat("bw",cfg.bw); prefs.putUChar("sf",cfg.sf); prefs.putUChar("cr",cfg.cr);
  prefs.putUChar("sw",cfg.sync); prefs.putChar("pw",cfg.power); prefs.putUShort("pre",cfg.preamble); prefs.putUShort("net",cfg.network);
  prefs.putInt("sck",cfg.sck); prefs.putInt("mi",cfg.miso); prefs.putInt("mo",cfg.mosi); prefs.putInt("cs",cfg.nss);
  prefs.putInt("d1",cfg.dio1); prefs.putInt("rst",cfg.rst); prefs.putInt("busy",cfg.busy); prefs.putInt("rx",cfg.rxen); prefs.putInt("tx",cfg.txen);
  prefs.putBytes("key",cfg.key,32);
#endif
}

static void loadConfig() {
#ifdef HOLDEN_TARGET_S2
  cfg.freq = prefs.getFloat("ccf", cfg.freq); cfg.br = prefs.getFloat("ccbr", cfg.br); cfg.dev = prefs.getFloat("ccdev", cfg.dev); cfg.bw = prefs.getFloat("ccbw", cfg.bw);
  cfg.csn = prefs.getInt("cccs", cfg.csn); cfg.gdo0 = prefs.getInt("ccg0", cfg.gdo0); cfg.gdo2 = prefs.getInt("ccg2", cfg.gdo2);
  if (!prefs.getBool("ccpins1", false)) { cfg.csn=CC_CSN; cfg.gdo0=CC_GDO0; cfg.gdo2=CC_GDO2; prefs.putBool("ccpins1", true); saveConfig(); }
#else
  cfg.freq=prefs.getFloat("f",cfg.freq); cfg.bw=prefs.getFloat("bw",cfg.bw); cfg.sf=prefs.getUChar("sf",cfg.sf); cfg.cr=prefs.getUChar("cr",cfg.cr);
  cfg.sync=prefs.getUChar("sw",cfg.sync); cfg.power=prefs.getChar("pw",cfg.power); cfg.preamble=prefs.getUShort("pre",cfg.preamble); cfg.network=prefs.getUShort("net",cfg.network);
  cfg.sck=prefs.getInt("sck",cfg.sck); cfg.miso=prefs.getInt("mi",cfg.miso); cfg.mosi=prefs.getInt("mo",cfg.mosi); cfg.nss=prefs.getInt("cs",cfg.nss);
  cfg.dio1=prefs.getInt("d1",cfg.dio1); cfg.rst=prefs.getInt("rst",cfg.rst); cfg.busy=prefs.getInt("busy",cfg.busy); cfg.rxen=prefs.getInt("rx",cfg.rxen); cfg.txen=prefs.getInt("tx",cfg.txen);
  if (prefs.getBytesLength("key") == 32) prefs.getBytes("key", cfg.key, 32); else { newKey(); prefs.putBytes("key", cfg.key, 32); }
  if (!prefs.getBool("bridgepins", false)) {
    cfg.sck=DEF_SCK; cfg.miso=DEF_MISO; cfg.mosi=DEF_MOSI; cfg.nss=DEF_NSS; cfg.rst=DEF_RST; cfg.dio1=DEF_DIO1; cfg.busy=DEF_BUSY; cfg.txen=DEF_TXEN; cfg.rxen=DEF_RXEN;
    prefs.putBool("bridgepins", true); saveConfig();
  }
#endif
}

#ifdef HOLDEN_TARGET_S2
void IRAM_ATTR gotCcPacket() { packetReady = true; }
static void stopRadio() {
  radioOk = false; packetReady = false; ccMuted = false;
  if (ccRadio) { delete ccRadio; ccRadio = nullptr; }
  if (ccModule) { delete ccModule; ccModule = nullptr; }
  SPI.end();
}
static bool startRadio() {
  stopRadio();
  Serial.printf("CC1101 wiring: CSN=%d GDO0=%d GDO2=%d SCK=%d MOSI=%d MISO=%d\n", cfg.csn, cfg.gdo0, cfg.gdo2, SCK, MOSI, MISO);
  SPI.begin(SCK, MISO, MOSI, cfg.csn);
  static SPISettings sp(4000000, MSBFIRST, SPI_MODE0);
  ccModule = new Module(cfg.csn, cfg.gdo0, RADIOLIB_NC, cfg.gdo2, SPI, sp);
  ccRadio = new CC1101(ccModule);
  int16_t r = ccRadio->begin(cfg.freq, cfg.br, cfg.dev, cfg.bw, 10, 16);
  if (r != RADIOLIB_ERR_NONE) { statusText = "CC1101 init failed " + String(r); Serial.println(statusText); return false; }
  ccRadio->setPacketReceivedAction(gotCcPacket);
  r = ccRadio->startReceive();
  if (r != RADIOLIB_ERR_NONE) { statusText = "CC1101 RX start failed " + String(r); Serial.println(statusText); return false; }
  radioOk = true; statusText = "CC1101 listening"; Serial.println(statusText); return true;
}
static void muteCc() {
  packetReady = false; ccResumeAt = 0;
  if (ccRadio && radioOk) ccRadio->standby();
  ccMuted = true; ccMutedSince = millis();
}
static void scheduleCcResume() { ccResumeAt = millis() + 40; }
static void serviceCcMute() {
  if (!ccMuted || !radioOk || !ccRadio) return;
  bool guardDone = ccResumeAt && (int32_t)(millis() - ccResumeAt) >= 0;
  bool failsafe = millis() - ccMutedSince > 5000;
  if (guardDone || failsafe) {
    packetReady = false;
    int16_t r = ccRadio->startReceive();
    if (r == RADIOLIB_ERR_NONE) { ccMuted = false; ccResumeAt = 0; statusText = "CC1101 listening"; }
    else statusText = "CC1101 resume failed " + String(r);
  }
}
static void serviceRadio() {
  if (!packetReady || !radioOk || !ccRadio || ccMuted) return;
  packetReady = false;
  size_t n = ccRadio->getPacketLength();
  if (n == 0 || n > MAX_BRIDGED_PAYLOAD) { dropCount++; statusText = "CC packet dropped: length " + String(n); ccRadio->startReceive(); return; }
  uint8_t data[MAX_BRIDGED_PAYLOAD];
  int16_t r = ccRadio->readData(data, n);
  if (r != RADIOLIB_ERR_NONE) { dropCount++; statusText = "CC1101 read failed " + String(r); ccRadio->startReceive(); return; }
  lastRssi = ccRadio->getRSSI(); ccRxCount++;
  muteCc();
  sendLink(LINK_DATA, data, (uint16_t)n); uartSentCount++;
  statusText = "CC packet sent to C3; RX muted for repeat";
}
static void handleLink(uint8_t type, const uint8_t *, uint16_t) {
  if (type == LINK_TX_BEGIN) { muteCc(); statusText = "DX-LR20 transmitting; CC1101 muted"; }
  else if (type == LINK_TX_END) { if (ccMuted) { scheduleCcResume(); statusText = "DX-LR20 done; CC1101 guard time"; } }
}
#else
static void stopRadio() {
  radioOk = false;
  if (dxRadio) { delete dxRadio; dxRadio = nullptr; }
  if (dxModule) { delete dxModule; dxModule = nullptr; }
  SPI.end();
}
static bool startRadio() {
  stopRadio();
  Serial.printf("DX-LR20 wiring: SCK=D8(%d) MISO=D9(%d) MOSI=D10(%d) NSS=D0(%d) RST=D1(%d) DIO1=D2(%d) BUSY=D3(%d) TXEN=D4(%d) RXEN=D5(%d)\n",
    cfg.sck,cfg.miso,cfg.mosi,cfg.nss,cfg.rst,cfg.dio1,cfg.busy,cfg.txen,cfg.rxen);
  SPI.begin(cfg.sck,cfg.miso,cfg.mosi,cfg.nss);
  static SPISettings sp(4000000,MSBFIRST,SPI_MODE0);
  dxModule = new Module(cfg.nss,cfg.dio1,cfg.rst,cfg.busy,SPI,sp);
  dxRadio = new LLCC68(dxModule); dxRadio->setRfSwitchPins(cfg.rxen,cfg.txen);
  int16_t r = dxRadio->begin(cfg.freq,cfg.bw,cfg.sf,cfg.cr,cfg.sync,cfg.power,cfg.preamble,0.0,false);
  if (r != RADIOLIB_ERR_NONE) { statusText = "DX-LR20 init failed " + String(r); Serial.println(statusText); return false; }
  radioOk = true; statusText = "DX-LR20 TX ready"; Serial.println(statusText); return true;
}
static size_t buildOutputFrame(const uint8_t *payload, size_t len, uint8_t *out) {
  if (len > MAX_BRIDGED_PAYLOAD) return 0;
  uint64_t ctr = prefs.getULong64("txctr", 0) + 1; prefs.putULong64("txctr", ctr);
  out[0]='H'; out[1]='L'; out[2]=1; put16(out+3,cfg.network); put32(out+5,repeaterId()); put64(out+9,ctr);
  out[17]=1; out[18]=(uint8_t)len; memcpy(out+HDR,payload,len);
  size_t signedLen = HDR + len; if (!mac16(out,signedLen,out+signedLen)) return 0; return signedLen + TAG;
}
static void repeatPacket(const uint8_t *payload, uint16_t len) {
  uartRxCount++; sendLink(LINK_TX_BEGIN); delay(8);
  if (!radioOk) { dropCount++; statusText="dropped: DX-LR20 not ready"; sendLink(LINK_TX_END); return; }
  if (len > MAX_BRIDGED_PAYLOAD) { dropCount++; statusText="dropped: CC packet too long"; sendLink(LINK_TX_END); return; }
  uint8_t frame[MAX_FRAME]; size_t n = buildOutputFrame(payload,len,frame);
  if (!n) { dropCount++; statusText="dropped: frame build failed"; sendLink(LINK_TX_END); return; }
  lastTxResult = dxRadio->transmit(frame,n);
  if (lastTxResult == RADIOLIB_ERR_NONE) { txCount++; statusText="repeated CC1101 packet over DX-LR20"; }
  else { dropCount++; statusText="DX-LR20 TX failed " + String(lastTxResult); }
  sendLink(LINK_TX_END);
}
static void serviceRadio() {}
static void handleLink(uint8_t type, const uint8_t *data, uint16_t len) { if (type == LINK_DATA) repeatPacket(data,len); }
#endif

static uint8_t linkBuf[HOLDEN_LINK_MAX + 7];
static uint16_t linkPos = 0;
static uint16_t linkExpected = 0;
static void resetLinkParser() { linkPos=0; linkExpected=0; }
static void serviceLink() {
  while (LinkSerial.available()) {
    uint8_t b = (uint8_t)LinkSerial.read();
    if (linkPos==0) { if (b==LINK_MAGIC0) linkBuf[linkPos++]=b; continue; }
    if (linkPos==1) { if (b==LINK_MAGIC1) linkBuf[linkPos++]=b; else resetLinkParser(); continue; }
    if (linkPos >= sizeof(linkBuf)) { resetLinkParser(); continue; }
    linkBuf[linkPos++] = b;
    if (linkPos==5) {
      uint16_t len=(uint16_t)linkBuf[3]|((uint16_t)linkBuf[4]<<8);
      if (len>HOLDEN_LINK_MAX) { resetLinkParser(); continue; }
      linkExpected=(uint16_t)(7+len);
    }
    if (linkExpected && linkPos==linkExpected) {
      uint8_t type=linkBuf[2]; uint16_t len=(uint16_t)linkBuf[3]|((uint16_t)linkBuf[4]<<8);
      uint16_t got=(uint16_t)linkBuf[5+len]|((uint16_t)linkBuf[6+len]<<8); uint16_t want=linkCrc(type,len,linkBuf+5);
      if (got==want) { linkLastRx = millis(); handleLink(type,linkBuf+5,len); }
      else { dropCount++; statusText="UART CRC error"; }
      resetLinkParser();
    }
  }
}

static String page() {
  String h; h.reserve(9400);
  bool linked = linkConnected();
  h += F("<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'><style>body{font-family:system-ui;background:#0b1020;color:#eef2ff;margin:20px}.w{max-width:900px;margin:auto}.c{background:#151c31;border:1px solid #2b3658;border-radius:16px;padding:18px;margin:14px 0}.g{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:9px}label{display:block;color:#b9c7ef;margin-top:8px}input{width:100%;box-sizing:border-box;padding:9px;background:#0e1529;color:#fff;border:1px solid #41507a;border-radius:8px}button{margin-top:12px;padding:10px 14px;background:#6d7dff;color:#fff;border:0;border-radius:9px;font-weight:700}.ok{color:#7bf0aa}.bad{color:#ff8a9a}code{background:#0a1123;padding:2px 5px;border-radius:5px}</style><div class=w><h1>Holden RF Repeater</h1><div class=c><h2>Status</h2>");
  h += "<p>Board: <b>" + String(BOARD_NAME) + "</b></p>";
  h += "<p>Wi-Fi: <b class='" + String(apOk ? "ok'>READY" : "bad'>FAILED") + "</b></p>";
  h += "<p>Radio: <b class='" + String(radioOk ? "ok'>READY" : "bad'>NOT READY") + "</b></p>";
#ifdef HOLDEN_TARGET_S2
  h += "<p>C3 link: <b class='" + String(linked ? "ok'>CONNECTED" : "bad'>DISCONNECTED") + "</b></p>";
#else
  h += "<p>S2 link: <b class='" + String(linked ? "ok'>CONNECTED" : "bad'>DISCONNECTED") + "</b></p>";
#endif
  h += "<p>" + statusText + "</p>";
#ifdef HOLDEN_TARGET_S2
  h += "<p>Role: <b>CC1101 receive &rarr; UART to C3</b></p><p>CC RX: " + String(ccRxCount) + " / UART sent: " + String(uartSentCount) + " / dropped: " + String(dropCount) + "</p><p>Last RSSI: " + String(lastRssi,1) + " dBm</p>";
  h += F("</div><div class=c><h2>Wiring</h2><p>CC1101 GDO0 &rarr; GPIO5, CSN &rarr; GPIO10, SCK &rarr; SCK, MOSI &rarr; MO, MISO/GDO1 &rarr; MI, GDO2 &rarr; GPIO6.</p><p>S2 RX(GPIO38) &larr; C3 D6 TX; S2 TX(GPIO39) &rarr; C3 D7 RX.</p><p>The CC1101 is automatically put in standby while the DX-LR20 retransmits, then RX resumes after a guard delay.</p></div>");
  h += F("<div class=c><h2>CC1101 receive configuration</h2><form method=post action=/save><div class=g>");
  auto f=[&](const char*l,const char*n,String v){h+="<div><label>"+String(l)+"</label><input name='"+n+"' value='"+v+"'></div>";};
  f("Frequency MHz","freq",String(cfg.freq,3)); f("Bitrate kbps","br",String(cfg.br,3)); f("Deviation kHz","dev",String(cfg.dev,3)); f("RX bandwidth kHz","bw",String(cfg.bw,1));
  h += F("</div><p>Current packet mode uses RadioLib CC1101 defaults for preamble/sync/CRC; frequency/bitrate/deviation/bandwidth are adjustable here.</p><button>Save + reboot</button></form><form method=post action=/reset><button>Factory reset</button></form></div></div>");
#else
  h += "<p>Role: <b>UART from S2 &rarr; DX-LR20 transmit</b></p><p>UART received: " + String(uartRxCount) + " / RF TX: " + String(txCount) + " / dropped: " + String(dropCount) + "</p>";
  h += F("</div><div class=c><h2>Wiring</h2><p>DX-LR20: NSS D0, RESET D1, DIO1 D2, BUSY D3, TXEN D4, RXEN D5, SCK D8, MISO D9, MOSI D10.</p><p>D6 TX &rarr; S2 RX; D7 RX &larr; S2 TX.</p><p>Every DX-LR20 transmission is wrapped with HL version, network ID, repeater ID, monotonic counter, TTL and HMAC-SHA256 tag. The S2 also mutes the CC1101 during TX.</p></div>");
  h += F("<div class=c><h2>DX-LR20 output configuration</h2><form method=post action=/save><label>256-bit output key</label><input name=key value='"); h += keyHex(); h += F("'><div class=g>");
  auto f=[&](const char*l,const char*n,String v){h+="<div><label>"+String(l)+"</label><input name='"+n+"' value='"+v+"'></div>";};
  f("Network ID","net",String(cfg.network)); f("Frequency MHz","freq",String(cfg.freq,3)); f("Bandwidth kHz","bw",String(cfg.bw,1)); f("SF","sf",String(cfg.sf)); f("CR denominator","cr",String(cfg.cr)); f("Sync word","sw",String(cfg.sync)); f("TX dBm","pw",String(cfg.power));
  h += F("</div><button>Save + reboot</button></form><form method=post action=/reset><button>Factory reset</button></form></div></div>");
#endif
  return h;
}

static void handleSave() {
#ifdef HOLDEN_TARGET_S2
  cfg.freq=argF("freq",cfg.freq); cfg.br=argF("br",cfg.br); cfg.dev=argF("dev",cfg.dev); cfg.bw=argF("bw",cfg.bw);
#else
  if (!server.hasArg("key") || !parseKey(server.arg("key"))) { server.send(400,"text/plain","Key must be 64 hex chars"); return; }
  cfg.network=(uint16_t)argI("net",cfg.network); cfg.freq=argF("freq",cfg.freq); cfg.bw=argF("bw",cfg.bw); cfg.sf=(uint8_t)argI("sf",cfg.sf); cfg.cr=(uint8_t)argI("cr",cfg.cr); cfg.sync=(uint8_t)argI("sw",cfg.sync); cfg.power=(int8_t)argI("pw",cfg.power);
#endif
  saveConfig(); server.send(200,"text/html","Saved; rebooting"); delay(300); ESP.restart();
}

static void startAP() {
  char suffix[7]; snprintf(suffix,sizeof(suffix),"%06lX",(unsigned long)(ESP.getEfuseMac()&0xFFFFFF));
  ssid="HOLDEN-LORA-"+String(BOARD_TAG)+"-"+suffix;
  WiFi.softAPdisconnect(true); delay(100); WiFi.mode(WIFI_AP); WiFi.setSleep(false);
  WiFi.softAPConfig(IPAddress(192,168,4,1),IPAddress(192,168,4,1),IPAddress(255,255,255,0));
  apOk=WiFi.softAP(ssid.c_str(),"repeater-setup",6,false,4); if(!apOk){WiFi.softAPdisconnect(true);delay(100);apOk=WiFi.softAP(ssid.c_str());}
  server.on("/",HTTP_GET,[](){server.send(200,"text/html",page());}); server.on("/save",HTTP_POST,handleSave);
  server.on("/reset",HTTP_POST,[](){prefs.clear();server.send(200,"text/plain","Reset; rebooting");delay(300);ESP.restart();});
  server.onNotFound([](){server.sendHeader("Location","/");server.send(302,"text/plain","");}); server.begin();
  Serial.printf("%s\nSSID: %s\nAP: %s\nUART RX=%d TX=%d @ %lu\n",BOARD_NAME,ssid.c_str(),apOk?"READY":"FAILED",UART_RX_PIN,UART_TX_PIN,(unsigned long)UART_BAUD);
}

void setup() {
  Serial.begin(115200); delay(400); prefs.begin("holden-lora",false); loadConfig();
  LinkSerial.begin(UART_BAUD,SERIAL_8N1,UART_RX_PIN,UART_TX_PIN);
  startAP(); statusText="setup ready; radio init delayed";
}

void loop() {
  server.handleClient(); serviceLink(); serviceHeartbeat(); serviceRadio();
#ifdef HOLDEN_TARGET_S2
  serviceCcMute();
#endif
  static uint32_t retryAt=0;
  if(!radioOk && millis()>2000 && millis()-retryAt>5000){retryAt=millis();startRadio();}
  delay(1);
}
