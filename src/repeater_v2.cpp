#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <RadioLib.h>
#include <mbedtls/md.h>
#include <esp_system.h>

#ifdef HOLDEN_TARGET_S2
static const char *BOARD_NAME = "Adafruit ESP32-S2 Feather";
static const char *BOARD_TAG = "S2";
static const int DEF_SCK=36, DEF_MISO=37, DEF_MOSI=35, DEF_NSS=10, DEF_DIO1=9, DEF_RST=6, DEF_BUSY=5, DEF_RXEN=11, DEF_TXEN=12;
#else
static const char *BOARD_NAME = "Seeed XIAO ESP32-C3";
static const char *BOARD_TAG = "C3";
static const int DEF_SCK=8, DEF_MISO=9, DEF_MOSI=10, DEF_NSS=5, DEF_DIO1=4, DEF_RST=3, DEF_BUSY=21, DEF_RXEN=6, DEF_TXEN=7;
#endif

static constexpr size_t HDR=19, TAG=16, MAX_FRAME=240;
static constexpr uint8_t MAGIC0='H', MAGIC1='L', VERSION=1;
static constexpr size_t O_NET=3, O_SENDER=5, O_COUNT=9, O_TTL=17, O_LEN=18;

struct Config {
  float freq=915.0f, bw=125.0f;
  uint8_t sf=9, cr=7, sync=0x12;
  int8_t power=22;
  uint16_t preamble=12, network=0x484C;
  int sck=DEF_SCK, miso=DEF_MISO, mosi=DEF_MOSI, nss=DEF_NSS, dio1=DEF_DIO1, rst=DEF_RST, busy=DEF_BUSY, rxen=DEF_RXEN, txen=DEF_TXEN;
  uint8_t key[32]{};
} cfg;

Preferences prefs;
WebServer server(80);
Module *mod=nullptr;
LLCC68 *radio=nullptr;
volatile bool packetReady=false;
bool apOk=false, radioOk=false;
String ssid, statusText="boot";
uint32_t rxCount=0, txCount=0, rejectCount=0;
float lastRssi=0, lastSnr=0;

static uint16_t u16(const uint8_t*p){return (uint16_t(p[0])<<8)|p[1];}
static uint32_t u32(const uint8_t*p){return (uint32_t(p[0])<<24)|(uint32_t(p[1])<<16)|(uint32_t(p[2])<<8)|p[3];}
static uint64_t u64(const uint8_t*p){uint64_t v=0;for(int i=0;i<8;i++)v=(v<<8)|p[i];return v;}

static void newKey(){for(int i=0;i<32;i+=4){uint32_t r=esp_random();memcpy(cfg.key+i,&r,4);}}
static String keyHex(){static const char*h="0123456789ABCDEF";String s;s.reserve(64);for(auto b:cfg.key){s+=h[b>>4];s+=h[b&15];}return s;}
static bool parseKey(String s){s.trim();s.replace(" ","");s.replace(":","");if(s.length()!=64)return false;for(int i=0;i<32;i++){auto n=[](char c)->int{if(c>='0'&&c<='9')return c-'0';if(c>='a'&&c<='f')return c-'a'+10;if(c>='A'&&c<='F')return c-'A'+10;return -1;};int a=n(s[i*2]),b=n(s[i*2+1]);if(a<0||b<0)return false;cfg.key[i]=(a<<4)|b;}return true;}
static bool mac16(const uint8_t*d,size_t n,uint8_t*out){auto md=mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);if(!md)return false;uint8_t sum[32];if(mbedtls_md_hmac(md,cfg.key,32,d,n,sum))return false;memcpy(out,sum,TAG);return true;}
static bool same(const uint8_t*a,const uint8_t*b,size_t n){uint8_t d=0;while(n--)d|=*a++^*b++;return d==0;}
static String ck(uint32_t sender){char b[13];snprintf(b,sizeof(b),"c%08lX",(unsigned long)sender);return b;}

static void save(){
  prefs.putFloat("f",cfg.freq);prefs.putFloat("bw",cfg.bw);prefs.putUChar("sf",cfg.sf);prefs.putUChar("cr",cfg.cr);prefs.putUChar("sw",cfg.sync);prefs.putChar("pw",cfg.power);prefs.putUShort("pre",cfg.preamble);prefs.putUShort("net",cfg.network);
  prefs.putInt("sck",cfg.sck);prefs.putInt("mi",cfg.miso);prefs.putInt("mo",cfg.mosi);prefs.putInt("cs",cfg.nss);prefs.putInt("d1",cfg.dio1);prefs.putInt("rst",cfg.rst);prefs.putInt("busy",cfg.busy);prefs.putInt("rx",cfg.rxen);prefs.putInt("tx",cfg.txen);prefs.putBytes("key",cfg.key,32);
}
static void load(){
  cfg.freq=prefs.getFloat("f",cfg.freq);cfg.bw=prefs.getFloat("bw",cfg.bw);cfg.sf=prefs.getUChar("sf",cfg.sf);cfg.cr=prefs.getUChar("cr",cfg.cr);cfg.sync=prefs.getUChar("sw",cfg.sync);cfg.power=prefs.getChar("pw",cfg.power);cfg.preamble=prefs.getUShort("pre",cfg.preamble);cfg.network=prefs.getUShort("net",cfg.network);
  cfg.sck=prefs.getInt("sck",cfg.sck);cfg.miso=prefs.getInt("mi",cfg.miso);cfg.mosi=prefs.getInt("mo",cfg.mosi);cfg.nss=prefs.getInt("cs",cfg.nss);cfg.dio1=prefs.getInt("d1",cfg.dio1);cfg.rst=prefs.getInt("rst",cfg.rst);cfg.busy=prefs.getInt("busy",cfg.busy);cfg.rxen=prefs.getInt("rx",cfg.rxen);cfg.txen=prefs.getInt("tx",cfg.txen);
  if(prefs.getBytesLength("key")==32)prefs.getBytes("key",cfg.key,32);else{newKey();prefs.putBytes("key",cfg.key,32);}
}

void IRAM_ATTR gotPacket(){packetReady=true;}
static void stopRadio(){radioOk=false;if(radio){delete radio;radio=nullptr;}if(mod){delete mod;mod=nullptr;}SPI.end();}
static bool startRadio(){
  stopRadio();SPI.begin(cfg.sck,cfg.miso,cfg.mosi,cfg.nss);static SPISettings sp(4000000,MSBFIRST,SPI_MODE0);mod=new Module(cfg.nss,cfg.dio1,cfg.rst,cfg.busy,SPI,sp);radio=new LLCC68(mod);radio->setRfSwitchPins(cfg.rxen,cfg.txen);
  int16_t r=radio->begin(cfg.freq,cfg.bw,cfg.sf,cfg.cr,cfg.sync,cfg.power,cfg.preamble,0.0,false);if(r!=RADIOLIB_ERR_NONE){statusText="radio init failed "+String(r);return false;}radio->setPacketReceivedAction(gotPacket);r=radio->startReceive();if(r!=RADIOLIB_ERR_NONE){statusText="RX start failed "+String(r);return false;}radioOk=true;statusText="radio ready";return true;
}

static bool valid(uint8_t*f,size_t n){
  if(n<HDR+TAG||n>MAX_FRAME||f[0]!=MAGIC0||f[1]!=MAGIC1||f[2]!=VERSION){rejectCount++;return false;}if(u16(f+O_NET)!=cfg.network){rejectCount++;return false;}size_t signedLen=HDR+f[O_LEN];if(signedLen+TAG!=n){rejectCount++;return false;}uint8_t m[TAG];if(!mac16(f,signedLen,m)||!same(m,f+signedLen,TAG)){rejectCount++;return false;}uint32_t sender=u32(f+O_SENDER);uint64_t count=u64(f+O_COUNT);String k=ck(sender);if(count<=prefs.getULong64(k.c_str(),0)||f[O_TTL]==0){rejectCount++;return false;}prefs.putULong64(k.c_str(),count);f[O_TTL]--;return mac16(f,signedLen,f+signedLen);
}
static void relay(){
  packetReady=false;if(!radioOk)return;size_t n=radio->getPacketLength();if(n==0||n>MAX_FRAME){radio->startReceive();return;}uint8_t f[MAX_FRAME];int16_t r=radio->readData(f,n);if(r!=RADIOLIB_ERR_NONE){radio->startReceive();return;}rxCount++;lastRssi=radio->getRSSI();lastSnr=radio->getSNR();if(valid(f,n)){delay(40+(esp_random()%120));r=radio->transmit(f,n);if(r==RADIOLIB_ERR_NONE){txCount++;statusText="forwarded";}else statusText="TX failed "+String(r);}radio->startReceive();
}

static long argI(const char*n,long d){return server.hasArg(n)?strtol(server.arg(n).c_str(),nullptr,0):d;}
static float argF(const char*n,float d){return server.hasArg(n)?server.arg(n).toFloat():d;}
static String page(){
  String h;h.reserve(8500);h+=F("<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'><style>body{font-family:system-ui;background:#0b1020;color:#eef2ff;margin:20px}.w{max-width:900px;margin:auto}.c{background:#151c31;border:1px solid #2b3658;border-radius:16px;padding:18px;margin:14px 0}.g{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:9px}label{display:block;color:#b9c7ef;margin-top:8px}input{width:100%;box-sizing:border-box;padding:9px;background:#0e1529;color:#fff;border:1px solid #41507a;border-radius:8px}button{margin-top:12px;padding:10px 14px;background:#6d7dff;color:#fff;border:0;border-radius:9px;font-weight:700}.ok{color:#7bf0aa}.bad{color:#ff8a9a}code{background:#0a1123;padding:2px 5px;border-radius:5px}</style><div class=w><h1>Holden LoRa Repeater</h1><div class=c><h2>Status</h2>");
  h+="<p>Board: <b>"+String(BOARD_NAME)+"</b></p><p>Setup Wi-Fi: <b class='"+(apOk?String("ok'>READY"):String("bad'>FAILED"))+"</b></p><p>Radio: <b class='"+(radioOk?String("ok'>READY"):String("bad'>NOT READY"))+"</b></p><p>SSID: <code>"+ssid+"</code></p><p>"+statusText+"</p><p>RX "+String(rxCount)+" / forwarded "+String(txCount)+" / rejected "+String(rejectCount)+"</p><p>RSSI/SNR "+String(lastRssi,1)+" dBm / "+String(lastSnr,1)+" dB</p></div>";
  h+=F("<div class=c><h2>Configuration</h2><form method=post action=/save><label>256-bit key</label><input name=key value='");h+=keyHex();h+=F("'><div class=g>");
  auto f=[&](const char*l,const char*n,String v){h+="<div><label>"+String(l)+"</label><input name='"+n+"' value='"+v+"'></div>";};
  f("Network ID","net",String(cfg.network));f("Frequency MHz","freq",String(cfg.freq,3));f("Bandwidth kHz","bw",String(cfg.bw,1));f("SF","sf",String(cfg.sf));f("CR denominator","cr",String(cfg.cr));f("Sync word","sw",String(cfg.sync));f("TX dBm","pw",String(cfg.power));f("SCK","sck",String(cfg.sck));f("MISO","mi",String(cfg.miso));f("MOSI","mo",String(cfg.mosi));f("NSS/CS","cs",String(cfg.nss));f("DIO1","d1",String(cfg.dio1));f("RESET","rst",String(cfg.rst));f("BUSY","busy",String(cfg.busy));f("RXEN","rx",String(cfg.rxen));f("TXEN","tx",String(cfg.txen));h+=F("</div><button>Save + reboot</button></form><form method=post action=/reset><button>Factory reset</button></form></div></div>");return h;
}
static void handleSave(){if(!server.hasArg("key")||!parseKey(server.arg("key"))){server.send(400,"text/plain","Key must be 64 hex chars");return;}cfg.network=argI("net",cfg.network);cfg.freq=argF("freq",cfg.freq);cfg.bw=argF("bw",cfg.bw);cfg.sf=argI("sf",cfg.sf);cfg.cr=argI("cr",cfg.cr);cfg.sync=argI("sw",cfg.sync);cfg.power=argI("pw",cfg.power);cfg.sck=argI("sck",cfg.sck);cfg.miso=argI("mi",cfg.miso);cfg.mosi=argI("mo",cfg.mosi);cfg.nss=argI("cs",cfg.nss);cfg.dio1=argI("d1",cfg.dio1);cfg.rst=argI("rst",cfg.rst);cfg.busy=argI("busy",cfg.busy);cfg.rxen=argI("rx",cfg.rxen);cfg.txen=argI("tx",cfg.txen);save();server.send(200,"text/html","Saved; rebooting");delay(300);ESP.restart();}
static void startAP(){
  char suf[7];snprintf(suf,sizeof(suf),"%06lX",(unsigned long)(ESP.getEfuseMac()&0xFFFFFF));ssid="HOLDEN-LORA-"+String(BOARD_TAG)+"-"+suf;WiFi.softAPdisconnect(true);delay(100);WiFi.mode(WIFI_AP);WiFi.setSleep(false);WiFi.softAPConfig(IPAddress(192,168,4,1),IPAddress(192,168,4,1),IPAddress(255,255,255,0));apOk=WiFi.softAP(ssid.c_str(),"repeater-setup",6,false,4);if(!apOk){WiFi.softAPdisconnect(true);delay(150);apOk=WiFi.softAP(ssid.c_str());}Serial.printf("%s\nSSID: %s\nAP: %s\nIP: %s\n",BOARD_NAME,ssid.c_str(),apOk?"READY":"FAILED",WiFi.softAPIP().toString().c_str());server.on("/",HTTP_GET,[](){server.send(200,"text/html",page());});server.on("/save",HTTP_POST,handleSave);server.on("/reset",HTTP_POST,[](){prefs.clear();server.send(200,"text/plain","Reset; rebooting");delay(300);ESP.restart();});server.onNotFound([](){server.sendHeader("Location","/");server.send(302,"text/plain","");});server.begin();
}

void setup(){Serial.begin(115200);delay(400);prefs.begin("holden-lora",false);load();startAP();statusText=apOk?"setup Wi-Fi ready; radio delayed":"setup Wi-Fi failed";}
void loop(){server.handleClient();if(packetReady)relay();static uint32_t retry=0;if(!radioOk&&millis()>3000&&millis()-retry>5000){retry=millis();startRadio();}delay(2);}
