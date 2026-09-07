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

static constexpr size_t OUTER_HDR = 19;
static constexpr size_t TAG_LEN = 16;
static constexpr size_t MAX_RF_FRAME = 240;
static constexpr size_t MAX_BRIDGED_PAYLOAD = MAX_RF_FRAME - OUTER_HDR - TAG_LEN;
static constexpr size_t MSG_HDR = 17;
static constexpr uint8_t MSG_VERSION = 1;

#ifdef HOLDEN_TARGET_S2
static const char *BOARD_NAME = "Adafruit ESP32-S2 Feather";
static const char *BOARD_TAG = "S2";
static const int UART_RX_PIN = RX;
static const int UART_TX_PIN = TX;
static const int CC_CSN = 10;
static const int CC_GDO0 = 5;
static const int CC_GDO2 = 6;

struct Config {
  float fskFreq = 915.0f;
  float fskBitrate = 4.8f;
  float fskDev = 5.0f;
  float fskBw = 58.0f;
} cfg;
#else
static const char *BOARD_NAME = "Seeed XIAO ESP32-C3";
static const char *BOARD_TAG = "C3";
static const int UART_TX_PIN = D6;
static const int UART_RX_PIN = D7;
static const int DX_SCK = D8;
static const int DX_MISO = D9;
static const int DX_MOSI = D10;
static const int DX_NSS = D0;
static const int DX_RST = D1;
static const int DX_DIO1 = D2;
static const int DX_BUSY = D3;
static const int DX_TXEN = D4;
static const int DX_RXEN = D5;

struct Config {
  float loraFreq = 915.0f;
  float loraBw = 125.0f;
  uint8_t sf = 9;
  uint8_t cr = 7;
  uint8_t sync = 0x12;
  int8_t power = 22;
  uint16_t preamble = 12;
  uint16_t network = 0x484C;
  uint8_t key[32]{};
} cfg;
#endif

Preferences prefs;
WebServer server(80);
HardwareSerial LinkSerial(1);

bool apOk = false;
bool radioOk = false;
bool prefsReady = false;
String ssid;
String statusText = "boot";
uint32_t dropCount = 0;
uint32_t linkLastRx = 0;
uint32_t linkLastHeartbeatTx = 0;

#ifdef HOLDEN_TARGET_S2
Module *ccModule = nullptr;
CC1101 *ccRadio = nullptr;
volatile bool ccPacketReady = false;
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

// Raw LoRa diagnostic state. This intentionally bypasses the Holden HM/HL
// protocol so the two DX-LR20 radios can be tested directly.
volatile bool dxPacketReady = false;
bool dxDiagRx = false;
uint32_t dxDiagRxCount = 0;
uint32_t dxDiagTxCount = 0;
float dxDiagLastRssi = 0.0f;
float dxDiagLastSnr = 0.0f;
String dxDiagLastText;
String dxDiagLastHex;
#endif

static bool linkConnected() {
  return linkLastRx != 0 && (uint32_t)(millis() - linkLastRx) < 3000;
}

static uint16_t crc16Update(uint16_t crc, uint8_t b) {
  crc ^= (uint16_t)b << 8;
  for (uint8_t i = 0; i < 8; ++i) {
    crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

static uint16_t linkCrc(uint8_t type, uint16_t len, const uint8_t *data) {
  uint16_t crc = 0xFFFF;
  crc = crc16Update(crc, type);
  crc = crc16Update(crc, (uint8_t)(len & 0xFF));
  crc = crc16Update(crc, (uint8_t)(len >> 8));
  for (uint16_t i = 0; data && i < len; ++i) crc = crc16Update(crc, data[i]);
  return crc;
}

static void sendLink(uint8_t type, const uint8_t *data = nullptr, uint16_t len = 0) {
  if (len > HOLDEN_LINK_MAX) return;
  uint16_t crc = linkCrc(type, len, data);
  LinkSerial.write(LINK_MAGIC0);
  LinkSerial.write(LINK_MAGIC1);
  LinkSerial.write(type);
  LinkSerial.write((uint8_t)(len & 0xFF));
  LinkSerial.write((uint8_t)(len >> 8));
  if (data && len) LinkSerial.write(data, len);
  LinkSerial.write((uint8_t)(crc & 0xFF));
  LinkSerial.write((uint8_t)(crc >> 8));
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
static void put16(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v;
}
static void put32(uint8_t *p, uint32_t v) {
  p[0]=(uint8_t)(v>>24); p[1]=(uint8_t)(v>>16); p[2]=(uint8_t)(v>>8); p[3]=(uint8_t)v;
}
static void put64(uint8_t *p, uint64_t v) {
  for (int i = 7; i >= 0; --i) { p[i] = (uint8_t)v; v >>= 8; }
}
static uint32_t get32(const uint8_t *p) {
  return ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3];
}
static uint64_t get64(const uint8_t *p) {
  uint64_t v = 0; for (int i=0;i<8;i++) v=(v<<8)|p[i]; return v;
}
static void newKey() {
  for (int i=0;i<32;i+=4) { uint32_t r=esp_random(); memcpy(cfg.key+i,&r,4); }
}
static String keyHex() {
  static const char *hex = "0123456789ABCDEF";
  String s; s.reserve(64);
  for (uint8_t b : cfg.key) { s += hex[b >> 4]; s += hex[b & 15]; }
  return s;
}
static bool parseKey(String s) {
  s.trim(); s.replace(" ", ""); s.replace(":", "");
  if (s.length() != 64) return false;
  auto nib=[](char c)->int {
    if(c>='0'&&c<='9')return c-'0';
    if(c>='a'&&c<='f')return c-'a'+10;
    if(c>='A'&&c<='F')return c-'A'+10;
    return -1;
  };
  for(int i=0;i<32;i++) {
    int a=nib(s[i*2]), b=nib(s[i*2+1]);
    if(a<0||b<0) return false;
    cfg.key[i]=(uint8_t)((a<<4)|b);
  }
  return true;
}
static bool hmac16(const uint8_t *data, size_t len, uint8_t out[TAG_LEN]) {
  const mbedtls_md_info_t *md=mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  if(!md) return false;
  uint8_t full[32];
  if(mbedtls_md_hmac(md,cfg.key,32,data,len,full)!=0) return false;
  memcpy(out,full,TAG_LEN); return true;
}
static bool ctEqual(const uint8_t *a,const uint8_t *b,size_t n) {
  uint8_t d=0; while(n--) d|=*a++^*b++; return d==0;
}
static uint32_t repeaterId() {
  uint64_t m=ESP.getEfuseMac(); return (uint32_t)(m^(m>>32));
}
static String senderCounterKey(uint32_t sender) {
  char b[13]; snprintf(b,sizeof(b),"m%08lX",(unsigned long)sender); return String(b);
}

static bool validInputMessage(const uint8_t *p, size_t n) {
  if (n < MSG_HDR + TAG_LEN) return false;
  if (p[0] != 'H' || p[1] != 'M' || p[2] != MSG_VERSION) return false;
  uint8_t nameLen = p[15];
  uint8_t textLen = p[16];
  size_t signedLen = MSG_HDR + (size_t)nameLen + (size_t)textLen;
  if (signedLen + TAG_LEN != n || nameLen == 0 || nameLen > 24 || textLen == 0) return false;
  uint8_t calc[TAG_LEN];
  if (!hmac16(p, signedLen, calc) || !ctEqual(calc, p + signedLen, TAG_LEN)) return false;
  uint32_t sender = get32(p + 3);
  uint64_t counter = get64(p + 7);
  String k = senderCounterKey(sender);
  uint64_t old = prefs.getULong64(k.c_str(), 0);
  if (counter <= old) return false;
  prefs.putULong64(k.c_str(), counter);
  return true;
}

static size_t buildRepeatedFrame(const uint8_t *payload, size_t len, uint8_t *out) {
  if (len > MAX_BRIDGED_PAYLOAD) return 0;
  uint64_t ctr = prefs.getULong64("outctr", 0) + 1;
  prefs.putULong64("outctr", ctr);
  out[0]='H'; out[1]='L'; out[2]=1;
  put16(out+3,cfg.network);
  put32(out+5,repeaterId());
  put64(out+9,ctr);
  out[17]=1;
  out[18]=(uint8_t)len;
  memcpy(out+OUTER_HDR,payload,len);
  size_t signedLen=OUTER_HDR+len;
  if(!hmac16(out,signedLen,out+signedLen)) return 0;
  return signedLen+TAG_LEN;
}

static String bytesToHex(const uint8_t *data, size_t len) {
  static const char *hex = "0123456789ABCDEF";
  String s; s.reserve(len * 3);
  for (size_t i = 0; i < len; ++i) {
    if (i) s += ' ';
    s += hex[data[i] >> 4];
    s += hex[data[i] & 0x0F];
  }
  return s;
}

static String bytesToSafeText(const uint8_t *data, size_t len) {
  String s; s.reserve(len);
  for (size_t i = 0; i < len; ++i) {
    char c = (char)data[i];
    if (c >= 32 && c <= 126 && c != '<' && c != '>' && c != '&') s += c;
    else s += '.';
  }
  return s;
}
#endif

static void saveConfig() {
#ifdef HOLDEN_TARGET_S2
  prefs.putFloat("fskf",cfg.fskFreq);
  prefs.putFloat("fskbr",cfg.fskBitrate);
  prefs.putFloat("fskdev",cfg.fskDev);
  prefs.putFloat("fskbw",cfg.fskBw);
#else
  prefs.putFloat("lf",cfg.loraFreq);
  prefs.putFloat("lbw",cfg.loraBw);
  prefs.putUChar("sf",cfg.sf);
  prefs.putUChar("cr",cfg.cr);
  prefs.putUChar("sync",cfg.sync);
  prefs.putChar("pwr",cfg.power);
  prefs.putUShort("pre",cfg.preamble);
  prefs.putUShort("net",cfg.network);
  prefs.putBytes("key",cfg.key,32);
#endif
}

static void loadConfig() {
#ifdef HOLDEN_TARGET_S2
  cfg.fskFreq=prefs.getFloat("fskf",cfg.fskFreq);
  cfg.fskBitrate=prefs.getFloat("fskbr",cfg.fskBitrate);
  cfg.fskDev=prefs.getFloat("fskdev",cfg.fskDev);
  cfg.fskBw=prefs.getFloat("fskbw",cfg.fskBw);
#else
  cfg.loraFreq=prefs.getFloat("lf",cfg.loraFreq);
  cfg.loraBw=prefs.getFloat("lbw",cfg.loraBw);
  cfg.sf=prefs.getUChar("sf",cfg.sf);
  cfg.cr=prefs.getUChar("cr",cfg.cr);
  cfg.sync=prefs.getUChar("sync",cfg.sync);
  cfg.power=prefs.getChar("pwr",cfg.power);
  cfg.preamble=prefs.getUShort("pre",cfg.preamble);
  cfg.network=prefs.getUShort("net",cfg.network);
  if(prefs.getBytesLength("key")==32) prefs.getBytes("key",cfg.key,32);
  else { newKey(); prefs.putBytes("key",cfg.key,32); }
#endif
}

#ifdef HOLDEN_TARGET_S2
void IRAM_ATTR gotCcPacket() { ccPacketReady=true; }

static void stopRadio() {
  radioOk=false; ccPacketReady=false; ccMuted=false;
  if(ccRadio){delete ccRadio;ccRadio=nullptr;}
  if(ccModule){delete ccModule;ccModule=nullptr;}
  SPI.end();
}

static bool startRadio() {
  stopRadio();
  SPI.begin(SCK,MISO,MOSI,CC_CSN);
  static SPISettings sp(4000000,MSBFIRST,SPI_MODE0);
  ccModule=new Module(CC_CSN,CC_GDO0,RADIOLIB_NC,CC_GDO2,SPI,sp);
  ccRadio=new CC1101(ccModule);
  int16_t r=ccRadio->begin(cfg.fskFreq,cfg.fskBitrate,cfg.fskDev,cfg.fskBw,10,16);
  if(r==RADIOLIB_ERR_NONE) r=ccRadio->setSyncWord(0x12,0xAD,0,false);
  if(r==RADIOLIB_ERR_NONE) r=ccRadio->setCrcFiltering(false);
  if(r!=RADIOLIB_ERR_NONE){statusText="CC1101 init failed "+String(r);return false;}
  ccRadio->setPacketReceivedAction(gotCcPacket);
  r=ccRadio->startReceive();
  if(r!=RADIOLIB_ERR_NONE){statusText="CC1101 RX failed "+String(r);return false;}
  radioOk=true; statusText="CC1101 listening"; return true;
}

static void muteCc() {
  ccPacketReady=false; ccResumeAt=0;
  if(ccRadio&&radioOk) ccRadio->standby();
  ccMuted=true; ccMutedSince=millis();
}

static void scheduleCcResume(){ccResumeAt=millis()+50;}

static void serviceCcMute(){
  if(!ccMuted||!radioOk||!ccRadio)return;
  bool guardDone=ccResumeAt && (int32_t)(millis()-ccResumeAt)>=0;
  bool failsafe=(uint32_t)(millis()-ccMutedSince)>5000;
  if(guardDone||failsafe){
    ccPacketReady=false;
    int16_t r=ccRadio->startReceive();
    if(r==RADIOLIB_ERR_NONE){ccMuted=false;ccResumeAt=0;statusText="CC1101 listening";}
    else statusText="CC1101 resume failed "+String(r);
  }
}

static void serviceRadio(){
  if(!ccPacketReady||!radioOk||!ccRadio||ccMuted)return;
  ccPacketReady=false;
  size_t n=ccRadio->getPacketLength();
  if(n==0||n>MAX_BRIDGED_PAYLOAD){dropCount++;statusText="CC packet length rejected";ccRadio->startReceive();return;}
  uint8_t data[MAX_BRIDGED_PAYLOAD];
  int16_t r=ccRadio->readData(data,n);
  if(r!=RADIOLIB_ERR_NONE){dropCount++;statusText="CC read failed "+String(r);ccRadio->startReceive();return;}
  lastRssi=ccRadio->getRSSI(); ccRxCount++;
  muteCc();
  sendLink(LINK_DATA,data,(uint16_t)n); uartSentCount++;
  statusText="packet sent to C3; CC1101 muted";
}

static void handleLink(uint8_t type,const uint8_t*,uint16_t){
  if(type==LINK_TX_BEGIN){muteCc();statusText="DX-LR20 transmitting; CC1101 muted";}
  else if(type==LINK_TX_END){if(ccMuted){scheduleCcResume();statusText="DX-LR20 done; guard delay";}}
}
#else
void IRAM_ATTR gotDxPacket() { dxPacketReady = true; }

static void stopRadio(){
  radioOk=false;
  dxDiagRx=false;
  dxPacketReady=false;
  if(dxRadio){dxRadio->clearPacketReceivedAction();delete dxRadio;dxRadio=nullptr;}
  if(dxModule){delete dxModule;dxModule=nullptr;}
  SPI.end();
}

static bool startRadio(){
  stopRadio();
  SPI.begin(DX_SCK,DX_MISO,DX_MOSI,DX_NSS);
  static SPISettings sp(4000000,MSBFIRST,SPI_MODE0);
  dxModule=new Module(DX_NSS,DX_DIO1,DX_RST,DX_BUSY,SPI,sp);
  dxRadio=new LLCC68(dxModule);
  dxRadio->setRfSwitchPins(DX_RXEN,DX_TXEN);
  int16_t r=dxRadio->begin(cfg.loraFreq,cfg.loraBw,cfg.sf,cfg.cr,cfg.sync,cfg.power,cfg.preamble,0.0,false);
  if(r==RADIOLIB_ERR_NONE) r=dxRadio->setCRC(true);
  if(r!=RADIOLIB_ERR_NONE){statusText="DX-LR20 init failed "+String(r);return false;}
  radioOk=true; statusText="DX-LR20 TX ready (LoRa CRC ON)"; return true;
}

static bool startDxDiagRx(){
  if(!radioOk||!dxRadio){statusText="diagnostic RX failed: DX-LR20 not ready";return false;}
  dxPacketReady=false;
  dxRadio->clearPacketReceivedAction();
  int16_t r=dxRadio->standby();
  if(r==RADIOLIB_ERR_NONE){
    dxRadio->setPacketReceivedAction(gotDxPacket);
    r=dxRadio->startReceive();
  }
  if(r!=RADIOLIB_ERR_NONE){
    dxRadio->clearPacketReceivedAction();
    dxDiagRx=false;
    statusText="diagnostic LoRa RX failed "+String(r);
    return false;
  }
  dxDiagRx=true;
  statusText="DX-LR20 diagnostic LoRa RX listening";
  return true;
}

static void stopDxDiagRx(){
  dxDiagRx=false;
  dxPacketReady=false;
  if(dxRadio){
    dxRadio->clearPacketReceivedAction();
    dxRadio->standby();
  }
  statusText="DX-LR20 TX ready (diagnostic RX stopped)";
}

static bool sendDxDiag(const String &input){
  if(!radioOk||!dxRadio){statusText="diagnostic TX failed: DX-LR20 not ready";return false;}
  String msg=input;
  if(msg.length()==0) msg="C3-LORA-TEST";
  if(msg.length()>180) msg=msg.substring(0,180);

  bool resumeRx=dxDiagRx;
  if(resumeRx) stopDxDiagRx();
  else dxRadio->standby();

  sendLink(LINK_TX_BEGIN); delay(10);
  lastTxResult=dxRadio->transmit((uint8_t*)msg.c_str(),msg.length());
  sendLink(LINK_TX_END);

  bool ok=(lastTxResult==RADIOLIB_ERR_NONE);
  if(ok){dxDiagTxCount++;statusText="raw LoRa diagnostic TX sent: "+msg;}
  else{statusText="raw LoRa diagnostic TX failed "+String(lastTxResult);}

  if(resumeRx){
    String txStatus=statusText;
    if(startDxDiagRx()) statusText=txStatus+"; RX resumed";
  }
  return ok;
}

static void repeatPacket(const uint8_t *payload,uint16_t len){
  uartRxCount++;
  if(dxDiagRx){
    dropCount++;
    statusText="normal repeat ignored while raw LoRa diagnostic RX is active";
    return;
  }
  if(!prefsReady||!validInputMessage(payload,len)){
    dropCount++; statusText="rejected unauthenticated/replayed endpoint packet"; return;
  }
  sendLink(LINK_TX_BEGIN); delay(10);
  if(!radioOk){dropCount++;statusText="dropped: DX-LR20 not ready";sendLink(LINK_TX_END);return;}
  uint8_t frame[MAX_RF_FRAME];
  size_t n=buildRepeatedFrame(payload,len,frame);
  if(!n){dropCount++;statusText="frame build failed";sendLink(LINK_TX_END);return;}
  lastTxResult=dxRadio->transmit(frame,n);
  if(lastTxResult==RADIOLIB_ERR_NONE){txCount++;statusText="repeated authenticated message";}
  else{dropCount++;statusText="DX-LR20 TX failed "+String(lastTxResult);}
  sendLink(LINK_TX_END);
}

static void serviceRadio(){
  if(!dxDiagRx||!dxPacketReady||!radioOk||!dxRadio)return;
  dxPacketReady=false;
  size_t n=dxRadio->getPacketLength();
  if(n==0||n>MAX_RF_FRAME){
    statusText="diagnostic RX packet length rejected";
    if(dxDiagRx)dxRadio->startReceive();
    return;
  }

  uint8_t data[MAX_RF_FRAME];
  int16_t r=dxRadio->readData(data,n);
  dxDiagLastRssi=dxRadio->getRSSI();
  dxDiagLastSnr=dxRadio->getSNR();
  if(r==RADIOLIB_ERR_NONE){
    dxDiagRxCount++;
    dxDiagLastText=bytesToSafeText(data,n);
    dxDiagLastHex=bytesToHex(data,n);
    statusText="raw LoRa diagnostic packet received";
  }else{
    statusText="diagnostic RX read failed "+String(r);
  }
  if(dxDiagRx)dxRadio->startReceive();
}

static void handleLink(uint8_t type,const uint8_t *data,uint16_t len){
  if(type==LINK_DATA) repeatPacket(data,len);
}
#endif

static uint8_t linkBuf[HOLDEN_LINK_MAX+7];
static uint16_t linkPos=0;
static uint16_t linkExpected=0;
static void resetLinkParser(){linkPos=0;linkExpected=0;}

static void serviceLink(){
  while(LinkSerial.available()){
    uint8_t b=(uint8_t)LinkSerial.read();
    if(linkPos==0){if(b==LINK_MAGIC0)linkBuf[linkPos++]=b;continue;}
    if(linkPos==1){if(b==LINK_MAGIC1)linkBuf[linkPos++]=b;else resetLinkParser();continue;}
    if(linkPos>=sizeof(linkBuf)){resetLinkParser();continue;}
    linkBuf[linkPos++]=b;
    if(linkPos==5){
      uint16_t len=(uint16_t)linkBuf[3]|((uint16_t)linkBuf[4]<<8);
      if(len>HOLDEN_LINK_MAX){resetLinkParser();continue;}
      linkExpected=(uint16_t)(7+len);
    }
    if(linkExpected&&linkPos==linkExpected){
      uint8_t type=linkBuf[2];
      uint16_t len=(uint16_t)linkBuf[3]|((uint16_t)linkBuf[4]<<8);
      uint16_t got=(uint16_t)linkBuf[5+len]|((uint16_t)linkBuf[6+len]<<8);
      uint16_t want=linkCrc(type,len,linkBuf+5);
      if(got==want){linkLastRx=millis();handleLink(type,linkBuf+5,len);}
      else{dropCount++;statusText="UART CRC error";}
      resetLinkParser();
    }
  }
}

static String page(){
  String h; h.reserve(11000);
  bool linked=linkConnected();
  h+=F("<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'><meta http-equiv=refresh content=4><style>body{font-family:system-ui;background:#09101f;color:#edf2ff;margin:20px}.w{max-width:850px;margin:auto}.c{background:#141d32;border:1px solid #2c395b;border-radius:16px;padding:18px;margin:14px 0}.g{display:grid;grid-template-columns:repeat(auto-fit,minmax(165px,1fr));gap:10px}label{display:block;color:#b6c3e5;margin-top:6px}input{width:100%;box-sizing:border-box;padding:9px;background:#0b1326;color:white;border:1px solid #405078;border-radius:8px}button{padding:10px 14px;margin-top:12px;border:0;border-radius:9px;background:#6678ff;color:white;font-weight:700}.ok{color:#72eda3}.bad{color:#ff8595}code{background:#091225;padding:2px 5px;border-radius:5px;overflow-wrap:anywhere}.warn{color:#ffd27a}</style><div class=w><h1>Holden RF Repeater</h1><div class=c><h2>Status</h2>");
  h+="<p>Board: <b>"+String(BOARD_NAME)+"</b></p>";
  h+="<p>Wi-Fi: <b class='"+String(apOk?"ok'>READY":"bad'>FAILED")+"</b></p>";
  h+="<p>Radio: <b class='"+String(radioOk?"ok'>READY":"bad'>NOT READY")+"</b></p>";
#ifdef HOLDEN_TARGET_S2
  h+="<p>C3 link: <b class='"+String(linked?"ok'>CONNECTED":"bad'>DISCONNECTED")+"</b></p>";
  h+="<p>Role: <b>CC1101 RX &rarr; UART &rarr; C3</b></p>";
  h+="<p>RF RX "+String(ccRxCount)+" / UART sent "+String(uartSentCount)+" / dropped "+String(dropCount)+"</p>";
  h+="<p>Last CC1101 RSSI: "+String(lastRssi,1)+" dBm</p>";
#else
  h+="<p>S2 link: <b class='"+String(linked?"ok'>CONNECTED":"bad'>DISCONNECTED")+"</b></p>";
  h+="<p>Role: <b>UART &rarr; authenticate &rarr; DX-LR20 LoRa TX</b></p>";
  h+="<p>UART RX "+String(uartRxCount)+" / repeated "+String(txCount)+" / dropped "+String(dropCount)+"</p>";
  h+="<p>DX diagnostic mode: <b>"+String(dxDiagRx?"RAW LoRa RX":"OFF / TX ready")+"</b></p>";
#endif
  h+="<p>"+statusText+"</p></div>";
#ifdef HOLDEN_TARGET_S2
  h+=F("<div class=c><h2>S2 wiring</h2><p>CC1101: GDO0=GPIO5, CSN=GPIO10, GDO2=GPIO6; SCK/MO/MI use the Feather SPI pins.</p><p>UART uses the Feather pins labeled <b>RX</b> and <b>TX</b>. C3 D6 TX goes to S2 RX; C3 D7 RX goes to S2 TX; grounds must be common.</p><p>The setup AP starts before UART, Preferences, or CC1101 initialization, so a radio/wiring failure cannot suppress Wi-Fi.</p></div>");
  h+=F("<div class=c><h2>CC1101 uplink</h2><form method=post action=/save><div class=g>");
  auto field=[&](const char*l,const char*n,String v){h+="<div><label>"+String(l)+"</label><input name='"+n+"' value='"+v+"'></div>";};
  field("FSK frequency MHz","f",String(cfg.fskFreq,3));
  field("Bitrate kbps","br",String(cfg.fskBitrate,3));
  field("Deviation kHz","dev",String(cfg.fskDev,3));
  field("RX bandwidth kHz","bw",String(cfg.fskBw,1));
  h+=F("</div><p>Sync word is fixed at <code>12 AD</code>. Hardware CRC is disabled; endpoint HMAC authenticates the packet.</p><button>Save + reboot</button></form></div>");
#else
  h+=F("<div class=c><h2>C3 wiring</h2><p>DX-LR20: NSS D0, RESET D1, DIO1 D2, BUSY D3, TXEN D4, RXEN D5, SCK D8, MISO D9, MOSI D10.</p><p>D6 TX &rarr; S2 RX; D7 RX &larr; S2 TX.</p></div>");
  h+=F("<div class=c><h2>Repeated LoRa output</h2><form method=post action=/save><label>Shared 256-bit key</label><input name=key value='");
  h+=keyHex(); h+=F("'><div class=g>");
  auto field=[&](const char*l,const char*n,String v){h+="<div><label>"+String(l)+"</label><input name='"+n+"' value='"+v+"'></div>";};
  field("Network ID","net",String(cfg.network));
  field("LoRa frequency MHz","f",String(cfg.loraFreq,3));
  field("Bandwidth kHz","bw",String(cfg.loraBw,1));
  field("SF","sf",String(cfg.sf));
  field("CR denominator","cr",String(cfg.cr));
  field("Sync word","sync",String(cfg.sync));
  field("TX dBm","pwr",String(cfg.power));
  h+=F("</div><p>LoRa CRC is explicitly ON. Copy the shared key and Network ID to both messenger endpoints. Receivers only display authenticated <b>repeated</b> LoRa frames from this C3.</p><button>Save + reboot</button></form></div>");

  h+=F("<div class=c><h2>DX-LR20 raw LoRa diagnostics</h2><p>This bypasses FSK, CC1101, Network ID, and authentication. It is only for proving DX-LR20 &harr; DX-LR20 LoRa communication.</p>");
  h+="<p>Settings: <code>"+String(cfg.loraFreq,3)+" MHz / BW "+String(cfg.loraBw,1)+" / SF"+String(cfg.sf)+" / CR4/"+String(cfg.cr)+" / preamble "+String(cfg.preamble)+" / sync 0x"+String(cfg.sync,HEX)+" / CRC ON</code></p>";
  h+="<p>Diagnostic TX: "+String(dxDiagTxCount)+" / RX: "+String(dxDiagRxCount)+"</p>";
  h+="<p>Last RX RSSI: "+String(dxDiagLastRssi,1)+" dBm / SNR: "+String(dxDiagLastSnr,1)+" dB</p>";
  h+="<p>Last RX text: <code>"+(dxDiagLastText.length()?dxDiagLastText:String("none"))+"</code></p>";
  h+="<p>Last RX hex: <code>"+(dxDiagLastHex.length()?dxDiagLastHex:String("none"))+"</code></p>";
  h+=F("<form method=post action=/diag-send><label>Raw LoRa test text</label><input name=msg value='C3-LORA-TEST' maxlength=180><button>Send raw LoRa test</button></form>");
  h+=F("<form method=post action=/diag-rx><button>");
  h+=dxDiagRx?"Stop raw LoRa RX":"Start raw LoRa RX";
  h+=F("</button></form><p class=warn>While raw diagnostic RX is active, normal repeated packets are intentionally not transmitted. Stop diagnostic RX to return to normal repeater operation.</p></div>");
#endif
  h+=F("<div class=c><form method=post action=/reset><button>Factory reset</button></form></div></div>");
  return h;
}

static void handleSave(){
  if(!prefsReady){server.send(503,"text/plain","Preferences not ready yet");return;}
#ifdef HOLDEN_TARGET_S2
  cfg.fskFreq=argF("f",cfg.fskFreq);
  cfg.fskBitrate=argF("br",cfg.fskBitrate);
  cfg.fskDev=argF("dev",cfg.fskDev);
  cfg.fskBw=argF("bw",cfg.fskBw);
#else
  if(!server.hasArg("key")||!parseKey(server.arg("key"))){server.send(400,"text/plain","Key must be 64 hex characters");return;}
  cfg.network=(uint16_t)argI("net",cfg.network);
  cfg.loraFreq=argF("f",cfg.loraFreq);
  cfg.loraBw=argF("bw",cfg.loraBw);
  cfg.sf=(uint8_t)argI("sf",cfg.sf);
  cfg.cr=(uint8_t)argI("cr",cfg.cr);
  cfg.sync=(uint8_t)argI("sync",cfg.sync);
  cfg.power=(int8_t)argI("pwr",cfg.power);
#endif
  saveConfig(); server.send(200,"text/html","Saved. Rebooting..."); delay(300); ESP.restart();
}

static void startAP(){
  char suffix[7]; snprintf(suffix,sizeof(suffix),"%06lX",(unsigned long)(ESP.getEfuseMac()&0xFFFFFF));
  ssid="HOLDEN-LORA-"+String(BOARD_TAG)+"-"+suffix;
  WiFi.softAPdisconnect(true); delay(150);
  WiFi.mode(WIFI_AP); delay(150);
  WiFi.softAPConfig(IPAddress(192,168,4,1),IPAddress(192,168,4,1),IPAddress(255,255,255,0));
  apOk=WiFi.softAP(ssid.c_str(),"repeater-setup",6,false,4);
  if(!apOk){
    WiFi.softAPdisconnect(true); delay(150);
    apOk=WiFi.softAP(ssid.c_str());
  }
  WiFi.setSleep(false);
  server.on("/",HTTP_GET,[](){server.send(200,"text/html",page());});
  server.on("/save",HTTP_POST,handleSave);
#ifndef HOLDEN_TARGET_S2
  server.on("/diag-send",HTTP_POST,[](){
    String msg=server.hasArg("msg")?server.arg("msg"):String("C3-LORA-TEST");
    bool ok=sendDxDiag(msg);
    server.send(ok?200:500,"text/plain",statusText);
  });
  server.on("/diag-rx",HTTP_POST,[](){
    if(dxDiagRx) stopDxDiagRx(); else startDxDiagRx();
    server.sendHeader("Location","/",true);
    server.send(303,"text/plain",statusText);
  });
#endif
  server.on("/reset",HTTP_POST,[](){
    if(prefsReady)prefs.clear();
    server.send(200,"text/plain","Reset. Rebooting..."); delay(300); ESP.restart();
  });
  server.onNotFound([](){server.sendHeader("Location","/");server.send(302,"text/plain","");});
  server.begin();
  Serial.printf("%s setup AP: %s (%s) IP 192.168.4.1\n",BOARD_NAME,ssid.c_str(),apOk?"READY":"FAILED");
}

void setup(){
  Serial.begin(115200); delay(120);
  startAP();
  statusText=apOk?"setup Wi-Fi ready; starting board services":"setup Wi-Fi failed";

  prefs.begin("holden-rpt",false); prefsReady=true; loadConfig();
  LinkSerial.begin(UART_BAUD,SERIAL_8N1,UART_RX_PIN,UART_TX_PIN);
  Serial.printf("UART link RX=%d TX=%d @ %lu\n",UART_RX_PIN,UART_TX_PIN,(unsigned long)UART_BAUD);
}

void loop(){
  server.handleClient();
  serviceLink();
  serviceHeartbeat();
  serviceRadio();
#ifdef HOLDEN_TARGET_S2
  serviceCcMute();
#endif
  static uint32_t retryAt=0;
  if(!radioOk&&millis()>1800&&(uint32_t)(millis()-retryAt)>5000){retryAt=millis();startRadio();}
  delay(1);
}
