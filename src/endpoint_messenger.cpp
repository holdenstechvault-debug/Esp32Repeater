#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <RadioLib.h>
#include <mbedtls/md.h>
#include <esp_system.h>

static constexpr size_t TAG_LEN = 16;
static constexpr size_t OUTER_HDR = 19;
static constexpr size_t MSG_HDR = 17;
static constexpr size_t MAX_RF_FRAME = 240;
static constexpr size_t MAX_INNER = MAX_RF_FRAME - OUTER_HDR - TAG_LEN;
static constexpr uint8_t MSG_VERSION = 1;
static constexpr uint8_t OUTER_VERSION = 1;
static constexpr uint8_t OUTER_REPEAT_MARKER = 1;
static constexpr size_t MAX_NAME_LEN = 20;
static constexpr size_t MAX_TEXT_LEN = 140;
static constexpr size_t HISTORY_SIZE = 24;

#ifdef HOLDEN_ENDPOINT_HELTEC
static const char *BOARD_NAME = "Heltec WiFi LoRa 32 V3";
static const char *NODE_TAG = "HELTEC";
static const char *DEFAULT_NAME = "GF";
static const int RADIO_SCK = 9;
static const int RADIO_MISO = 11;
static const int RADIO_MOSI = 10;
static const int RADIO_NSS = 8;
static const int RADIO_RST = 12;
static const int RADIO_DIO1 = 14;
static const int RADIO_BUSY = 13;
static constexpr float RADIO_TCXO = 1.8f;
#else
static const char *BOARD_NAME = "Home XIAO ESP32-C3 + DX-LR20";
static const char *NODE_TAG = "HOME";
static const char *DEFAULT_NAME = "Home";
static const int RADIO_SCK = D8;
static const int RADIO_MISO = D9;
static const int RADIO_MOSI = D10;
static const int RADIO_NSS = D0;
static const int RADIO_RST = D1;
static const int RADIO_DIO1 = D2;
static const int RADIO_BUSY = D3;
static const int RADIO_TXEN = D4;
static const int RADIO_RXEN = D5;
static constexpr float RADIO_TCXO = 0.0f;
#endif

struct Config {
  uint16_t network = 0x484C;
  uint8_t key[32]{};
  String name = DEFAULT_NAME;

  float loraFreq = 915.0f;
  float loraBw = 125.0f;
  uint8_t sf = 9;
  uint8_t cr = 7;
  uint8_t sync = 0x12;
  uint16_t loraPreamble = 12;

  float fskFreq = 915.0f;
  float fskBitrate = 4.8f;
  float fskDev = 5.0f;
  float fskBw = 58.6f;
  int8_t fskPower = 10;
} cfg;

struct MessageItem {
  bool used = false;
  uint32_t sender = 0;
  uint64_t counter = 0;
  String name;
  String text;
  float rssi = 0;
  float snr = 0;
};

Preferences prefs;
WebServer server(80);
String ssid;
bool apOk = false;
bool prefsReady = false;
bool radioOk = false;
String statusText = "boot";

Module *radioModule = nullptr;
#ifdef HOLDEN_ENDPOINT_HELTEC
SX1262 *radio = nullptr;
#else
LLCC68 *radio = nullptr;
#endif
volatile bool packetReady = false;

enum RadioMode : uint8_t { RADIO_NONE, RADIO_LORA_RX, RADIO_FSK_TX };
RadioMode radioMode = RADIO_NONE;

MessageItem history[HISTORY_SIZE];
size_t historyHead = 0;
size_t historyCount = 0;
uint32_t acceptedCount = 0;
uint32_t rejectedCount = 0;
uint32_t txCount = 0;
uint32_t txFailCount = 0;
float lastRssi = 0;
float lastSnr = 0;
uint32_t nodeId = 0;

static uint16_t get16(const uint8_t *p) {
  return ((uint16_t)p[0] << 8) | p[1];
}
static uint32_t get32(const uint8_t *p) {
  return ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3];
}
static uint64_t get64(const uint8_t *p) {
  uint64_t v=0; for(int i=0;i<8;i++)v=(v<<8)|p[i]; return v;
}
static void put32(uint8_t *p,uint32_t v) {
  p[0]=(uint8_t)(v>>24);p[1]=(uint8_t)(v>>16);p[2]=(uint8_t)(v>>8);p[3]=(uint8_t)v;
}
static void put64(uint8_t *p,uint64_t v) {
  for(int i=7;i>=0;--i){p[i]=(uint8_t)v;v>>=8;}
}

static bool ctEqual(const uint8_t *a,const uint8_t *b,size_t n){
  uint8_t d=0; while(n--)d|=*a++^*b++; return d==0;
}

static bool hmac16(const uint8_t *data,size_t len,uint8_t out[TAG_LEN]){
  const mbedtls_md_info_t *md=mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  if(!md)return false;
  uint8_t full[32];
  if(mbedtls_md_hmac(md,cfg.key,32,data,len,full)!=0)return false;
  memcpy(out,full,TAG_LEN);return true;
}

static bool keyConfigured(){
  uint8_t v=0;for(uint8_t b:cfg.key)v|=b;return v!=0;
}

static String keyHex(){
  static const char*h="0123456789ABCDEF";
  String s;s.reserve(64);for(uint8_t b:cfg.key){s+=h[b>>4];s+=h[b&15];}return s;
}

static bool parseKey(String s){
  s.trim();s.replace(" ","");s.replace(":","");if(s.length()!=64)return false;
  auto nib=[](char c)->int{if(c>='0'&&c<='9')return c-'0';if(c>='a'&&c<='f')return c-'a'+10;if(c>='A'&&c<='F')return c-'A'+10;return -1;};
  for(int i=0;i<32;i++){int a=nib(s[i*2]),b=nib(s[i*2+1]);if(a<0||b<0)return false;cfg.key[i]=(uint8_t)((a<<4)|b);}return true;
}

static String htmlEscape(const String &s){
  String o;o.reserve(s.length()+8);
  for(size_t i=0;i<s.length();++i){
    char c=s[i];
    if(c=='&')o+=F("&amp;");
    else if(c=='<')o+=F("&lt;");
    else if(c=='>')o+=F("&gt;");
    else if(c=='\"')o+=F("&quot;");
    else if(c=='\'')o+=F("&#39;");
    else o+=c;
  }
  return o;
}

static void saveConfig(){
  prefs.putUShort("net",cfg.network);
  prefs.putBytes("key",cfg.key,32);
  prefs.putString("name",cfg.name);
  prefs.putFloat("lf",cfg.loraFreq);
  prefs.putFloat("lbw",cfg.loraBw);
  prefs.putUChar("sf",cfg.sf);
  prefs.putUChar("cr",cfg.cr);
  prefs.putUChar("sync",cfg.sync);
  prefs.putUShort("lpre",cfg.loraPreamble);
  prefs.putFloat("ff",cfg.fskFreq);
  prefs.putFloat("fbr",cfg.fskBitrate);
  prefs.putFloat("fdev",cfg.fskDev);
  prefs.putFloat("fbw",cfg.fskBw);
  prefs.putChar("fpwr",cfg.fskPower);
}

static void loadConfig(){
  cfg.network=prefs.getUShort("net",cfg.network);
  if(prefs.getBytesLength("key")==32)prefs.getBytes("key",cfg.key,32);
  cfg.name=prefs.getString("name",cfg.name);
  if(cfg.name.length()==0)cfg.name=DEFAULT_NAME;
  cfg.loraFreq=prefs.getFloat("lf",cfg.loraFreq);
  cfg.loraBw=prefs.getFloat("lbw",cfg.loraBw);
  cfg.sf=prefs.getUChar("sf",cfg.sf);
  cfg.cr=prefs.getUChar("cr",cfg.cr);
  cfg.sync=prefs.getUChar("sync",cfg.sync);
  cfg.loraPreamble=prefs.getUShort("lpre",cfg.loraPreamble);
  cfg.fskFreq=prefs.getFloat("ff",cfg.fskFreq);
  cfg.fskBitrate=prefs.getFloat("fbr",cfg.fskBitrate);
  cfg.fskDev=prefs.getFloat("fdev",cfg.fskDev);
  cfg.fskBw=prefs.getFloat("fbw",cfg.fskBw);
  cfg.fskPower=prefs.getChar("fpwr",cfg.fskPower);
}

void IRAM_ATTR gotPacket(){packetReady=true;}

static void destroyRadio(){
  packetReady=false;radioOk=false;radioMode=RADIO_NONE;
  if(radio){delete radio;radio=nullptr;}
  if(radioModule){delete radioModule;radioModule=nullptr;}
  SPI.end();
}

static void createRadio(){
  SPI.begin(RADIO_SCK,RADIO_MISO,RADIO_MOSI,RADIO_NSS);
  static SPISettings sp(4000000,MSBFIRST,SPI_MODE0);
  radioModule=new Module(RADIO_NSS,RADIO_DIO1,RADIO_RST,RADIO_BUSY,SPI,sp);
#ifdef HOLDEN_ENDPOINT_HELTEC
  radio=new SX1262(radioModule);
#else
  radio=new LLCC68(radioModule);
  radio->setRfSwitchPins(RADIO_RXEN,RADIO_TXEN);
#endif
}

static bool startLoRaRx(){
  destroyRadio();createRadio();
  int16_t r=radio->begin(cfg.loraFreq,cfg.loraBw,cfg.sf,cfg.cr,cfg.sync,10,cfg.loraPreamble,RADIO_TCXO,false);
  if(r!=RADIOLIB_ERR_NONE){statusText="LoRa init failed "+String(r);return false;}
  radio->setPacketReceivedAction(gotPacket);
  r=radio->startReceive();
  if(r!=RADIOLIB_ERR_NONE){statusText="LoRa RX failed "+String(r);return false;}
  radioOk=true;radioMode=RADIO_LORA_RX;statusText="listening for repeated LoRa only";return true;
}

static bool startFskTx(){
  destroyRadio();createRadio();
  int16_t r=radio->beginFSK(cfg.fskFreq,cfg.fskBitrate,cfg.fskDev,cfg.fskBw,cfg.fskPower,16,RADIO_TCXO,false);
  if(r==RADIOLIB_ERR_NONE){uint8_t sw[2]={0x12,0xAD};r=radio->setSyncWord(sw,2);}
  if(r==RADIOLIB_ERR_NONE)r=radio->setCRC(0);
  if(r!=RADIOLIB_ERR_NONE){statusText="FSK init failed "+String(r);return false;}
  radioOk=true;radioMode=RADIO_FSK_TX;return true;
}

static void addHistory(uint32_t sender,uint64_t counter,const String &name,const String &text,float rssi,float snr){
  MessageItem &m=history[historyHead];
  m.used=true;m.sender=sender;m.counter=counter;m.name=name;m.text=text;m.rssi=rssi;m.snr=snr;
  historyHead=(historyHead+1)%HISTORY_SIZE;
  if(historyCount<HISTORY_SIZE)historyCount++;
}

static bool verifyInnerMessage(const uint8_t *p,size_t n,String &name,String &text,uint32_t &sender,uint64_t &counter){
  if(n<MSG_HDR+TAG_LEN)return false;
  if(p[0]!='H'||p[1]!='M'||p[2]!=MSG_VERSION)return false;
  uint8_t nameLen=p[15],textLen=p[16];
  size_t signedLen=MSG_HDR+(size_t)nameLen+(size_t)textLen;
  if(nameLen==0||nameLen>MAX_NAME_LEN||textLen==0||textLen>MAX_TEXT_LEN||signedLen+TAG_LEN!=n)return false;
  uint8_t mac[TAG_LEN];
  if(!hmac16(p,signedLen,mac)||!ctEqual(mac,p+signedLen,TAG_LEN))return false;
  sender=get32(p+3);counter=get64(p+7);
  name=""; text=""; name.reserve(nameLen); text.reserve(textLen);
  for(uint8_t i=0;i<nameLen;i++) name+=(char)p[MSG_HDR+i];
  for(uint8_t i=0;i<textLen;i++) text+=(char)p[MSG_HDR+nameLen+i];
  return true;
}

static bool acceptRepeatedFrame(const uint8_t *f,size_t n,float rssi,float snr){
  if(!keyConfigured()||n<OUTER_HDR+TAG_LEN||n>MAX_RF_FRAME)return false;
  if(f[0]!='H'||f[1]!='L'||f[2]!=OUTER_VERSION)return false;
  if(get16(f+3)!=cfg.network)return false;
  if(f[17]!=OUTER_REPEAT_MARKER)return false;
  uint8_t payloadLen=f[18];
  size_t signedLen=OUTER_HDR+(size_t)payloadLen;
  if(signedLen+TAG_LEN!=n)return false;
  uint8_t mac[TAG_LEN];
  if(!hmac16(f,signedLen,mac)||!ctEqual(mac,f+signedLen,TAG_LEN))return false;
  uint64_t outerCounter=get64(f+9);
  uint64_t old=prefs.getULong64("rxout",0);
  if(outerCounter<=old)return false;

  String name,text;uint32_t sender=0;uint64_t msgCounter=0;
  if(!verifyInnerMessage(f+OUTER_HDR,payloadLen,name,text,sender,msgCounter))return false;

  prefs.putULong64("rxout",outerCounter);
  addHistory(sender,msgCounter,name,text,rssi,snr);
  acceptedCount++;
  return true;
}

static void serviceReceive(){
  if(!packetReady||!radioOk||radioMode!=RADIO_LORA_RX||!radio)return;
  packetReady=false;
  size_t n=radio->getPacketLength();
  if(n==0||n>MAX_RF_FRAME){rejectedCount++;radio->startReceive();return;}
  uint8_t frame[MAX_RF_FRAME];
  int16_t r=radio->readData(frame,n);
  lastRssi=radio->getRSSI();lastSnr=radio->getSNR();
  if(r==RADIOLIB_ERR_NONE&&acceptRepeatedFrame(frame,n,lastRssi,lastSnr))statusText="received authenticated repeated message";
  else{rejectedCount++;statusText="ignored non-repeated/invalid RF packet";}
  radio->startReceive();
}

static size_t buildInnerMessage(const String &text,uint8_t *out){
  String name=cfg.name;name.trim();
  if(name.length()==0)name=DEFAULT_NAME;
  if(name.length()>MAX_NAME_LEN)name=name.substring(0,MAX_NAME_LEN);
  String msg=text;
  if(msg.length()>MAX_TEXT_LEN)msg=msg.substring(0,MAX_TEXT_LEN);
  if(msg.length()==0)return 0;

  uint64_t counter=prefs.getULong64("txmsg",0)+1;
  prefs.putULong64("txmsg",counter);
  out[0]='H';out[1]='M';out[2]=MSG_VERSION;
  put32(out+3,nodeId);put64(out+7,counter);
  out[15]=(uint8_t)name.length();out[16]=(uint8_t)msg.length();
  memcpy(out+MSG_HDR,name.c_str(),name.length());
  memcpy(out+MSG_HDR+name.length(),msg.c_str(),msg.length());
  size_t signedLen=MSG_HDR+name.length()+msg.length();
  if(!hmac16(out,signedLen,out+signedLen))return 0;
  return signedLen+TAG_LEN;
}

static bool sendMessage(const String &text){
  if(!prefsReady||!keyConfigured()){statusText="set the shared key first";return false;}
  uint8_t packet[MAX_INNER];
  size_t n=buildInnerMessage(text,packet);
  if(!n||n>MAX_INNER){statusText="message too long";return false;}
  statusText="switching to FSK uplink";
  if(!startFskTx()){txFailCount++;startLoRaRx();return false;}
  int16_t r=radio->transmit(packet,n);
  if(r==RADIOLIB_ERR_NONE){txCount++;statusText="FSK sent; waiting for repeated LoRa echo";}
  else{txFailCount++;statusText="FSK TX failed "+String(r);}
  bool sent=(r==RADIOLIB_ERR_NONE);
  delay(15);
  if(!startLoRaRx()){statusText+="; LoRa RX restart failed";}
  return sent;
}

static String messagesHtml(){
  String h;h.reserve(5000);
  if(historyCount==0)return F("<div class=empty>No authenticated repeated messages yet.</div>");
  size_t first=(historyHead+HISTORY_SIZE-historyCount)%HISTORY_SIZE;
  for(size_t k=0;k<historyCount;k++){
    size_t idx=(first+k)%HISTORY_SIZE;
    MessageItem &m=history[idx];if(!m.used)continue;
    bool mine=m.sender==nodeId;
    h+="<div class='msg "+String(mine?"mine":"other")+"'><div class=who>"+htmlEscape(m.name)+(mine?" (me)":"")+"</div><div class=text>"+htmlEscape(m.text)+"</div><div class=meta>repeated LoRa &bull; "+String(m.rssi,1)+" dBm &bull; "+String(m.snr,1)+" dB</div></div>";
  }
  return h;
}

static String statusJson(){
  String s="{";
  s+="\"radio\":"+String(radioOk?"true":"false")+",";
  s+="\"mode\":\""+String(radioMode==RADIO_LORA_RX?"LoRa RX (repeated only)":radioMode==RADIO_FSK_TX?"FSK TX":"offline")+"\",";
  s+="\"accepted\":"+String(acceptedCount)+",";
  s+="\"rejected\":"+String(rejectedCount)+",";
  s+="\"tx\":"+String(txCount)+",";
  s+="\"status\":\"";
  String safe=statusText;safe.replace("\\","\\\\");safe.replace("\"","\\\"");
  s+=safe+"\"}";return s;
}

static String page(){
  String h;h.reserve(12000);
  h+=F("<!doctype html><html><head><meta name=viewport content='width=device-width,initial-scale=1'><title>Holden Messenger</title><style>body{font-family:system-ui;background:#08101e;color:#edf3ff;margin:0}.wrap{max-width:760px;margin:auto;padding:18px}.card{background:#131d31;border:1px solid #293958;border-radius:17px;padding:16px;margin:13px 0}h1{margin:.2rem 0}.sub{color:#9fb0d3}.status{font-size:.92rem;color:#b7c6e6}.msgs{min-height:180px}.msg{padding:11px 13px;border-radius:14px;margin:9px 0;max-width:82%;background:#202c46}.msg.mine{margin-left:auto;background:#283a66}.who{font-weight:700;font-size:.85rem}.text{font-size:1.05rem;white-space:pre-wrap;overflow-wrap:anywhere}.meta{font-size:.72rem;color:#9eadd0;margin-top:5px}.empty{color:#8f9dbc;padding:20px 4px}textarea,input{width:100%;box-sizing:border-box;background:#091426;color:#fff;border:1px solid #405379;border-radius:10px;padding:10px}button{background:#667cff;color:white;border:0;border-radius:10px;padding:11px 16px;font-weight:750;margin-top:9px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:9px}label{display:block;color:#b8c5e3;font-size:.85rem;margin:5px 0}.good{color:#72efa5}.warn{color:#ffd07a}details summary{cursor:pointer;font-weight:700}</style></head><body><div class=wrap><h1>Holden Messenger</h1><div class=sub>");
  h+=String(BOARD_NAME)+" &bull; node <code>"+String(nodeId,HEX)+"</code></div>";
  h+=F("<div class=card><div id=status class=status>Loading status...</div></div><div class='card msgs' id=messages></div><div class=card><form id=sendForm><label>Message</label><textarea id=text name=text maxlength=140 rows=3 placeholder='Type a message...'></textarea><button>Send to repeater</button></form><div id=sendResult class=status></div><p class=sub>Messages are <b>not</b> added locally when you press send. They appear only after the repeater receives the FSK uplink and sends back a valid authenticated LoRa repeat.</p></div>");
  h+=F("<details class=card><summary>Radio & security settings</summary><form method=post action=/config><label>Name</label><input name=name value='");h+=htmlEscape(cfg.name);h+=F("'><label>Shared 256-bit key</label><input name=key value='");h+=keyHex();h+=F("'><div class=grid>");
  auto field=[&](const char*l,const char*n,String v){h+="<div><label>"+String(l)+"</label><input name='"+n+"' value='"+v+"'></div>";};
  field("Network ID","net",String(cfg.network));
  field("Repeated LoRa MHz","lf",String(cfg.loraFreq,3));
  field("LoRa BW kHz","lbw",String(cfg.loraBw,1));
  field("LoRa SF","sf",String(cfg.sf));
  field("LoRa CR","cr",String(cfg.cr));
  field("FSK uplink MHz","ff",String(cfg.fskFreq,3));
  field("FSK bitrate kbps","fbr",String(cfg.fskBitrate,3));
  field("FSK deviation kHz","fdev",String(cfg.fskDev,3));
  field("FSK TX dBm","fpwr",String(cfg.fskPower));
  h+=F("</div><button>Save + reboot</button></form><p class=sub>Use the C3 repeater page's key and Network ID here. FSK settings must match the S2 CC1101. LoRa settings must match the repeater DX-LR20 output.</p></details>");
  h+=F("<script>async function poll(){try{let m=await fetch('/messages');document.getElementById('messages').innerHTML=await m.text();let r=await fetch('/status');let j=await r.json();document.getElementById('status').innerHTML='<b class='+(j.radio?'good':'warn')+'>'+(j.radio?'RADIO READY':'RADIO NOT READY')+'</b> &bull; '+j.mode+' &bull; RX '+j.accepted+' &bull; ignored '+j.rejected+' &bull; TX '+j.tx+'<br>'+j.status;}catch(e){}}setInterval(poll,1400);poll();document.getElementById('sendForm').addEventListener('submit',async e=>{e.preventDefault();let t=document.getElementById('text');let out=document.getElementById('sendResult');out.textContent='Sending...';let r=await fetch('/send',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams({text:t.value})});out.textContent=await r.text();if(r.ok)t.value='';setTimeout(poll,300);});</script></div></body></html>");
  return h;
}

static long argI(const char*n,long d){return server.hasArg(n)?strtol(server.arg(n).c_str(),nullptr,0):d;}
static float argF(const char*n,float d){return server.hasArg(n)?server.arg(n).toFloat():d;}

static void handleConfig(){
  if(!prefsReady){server.send(503,"text/plain","not ready");return;}
  if(!server.hasArg("key")||!parseKey(server.arg("key"))){server.send(400,"text/plain","Key must be exactly 64 hex characters");return;}
  cfg.name=server.arg("name");cfg.name.trim();if(cfg.name.length()==0)cfg.name=DEFAULT_NAME;if(cfg.name.length()>MAX_NAME_LEN)cfg.name=cfg.name.substring(0,MAX_NAME_LEN);
  cfg.network=(uint16_t)argI("net",cfg.network);
  cfg.loraFreq=argF("lf",cfg.loraFreq);cfg.loraBw=argF("lbw",cfg.loraBw);cfg.sf=(uint8_t)argI("sf",cfg.sf);cfg.cr=(uint8_t)argI("cr",cfg.cr);
  cfg.fskFreq=argF("ff",cfg.fskFreq);cfg.fskBitrate=argF("fbr",cfg.fskBitrate);cfg.fskDev=argF("fdev",cfg.fskDev);cfg.fskPower=(int8_t)argI("fpwr",cfg.fskPower);
  saveConfig();server.send(200,"text/html","Saved. Rebooting...");delay(300);ESP.restart();
}

static void startAP(){
  char suffix[7];snprintf(suffix,sizeof(suffix),"%06lX",(unsigned long)(ESP.getEfuseMac()&0xFFFFFF));
  ssid="HOLDEN-MSG-"+String(NODE_TAG)+"-"+suffix;
  WiFi.softAPdisconnect(true);delay(120);WiFi.mode(WIFI_AP);delay(120);
  WiFi.softAPConfig(IPAddress(192,168,4,1),IPAddress(192,168,4,1),IPAddress(255,255,255,0));
  apOk=WiFi.softAP(ssid.c_str(),"holden-messenger",6,false,4);
  if(!apOk){WiFi.softAPdisconnect(true);delay(120);apOk=WiFi.softAP(ssid.c_str());}
  WiFi.setSleep(false);
  server.on("/",HTTP_GET,[](){server.send(200,"text/html",page());});
  server.on("/messages",HTTP_GET,[](){server.send(200,"text/html",messagesHtml());});
  server.on("/status",HTTP_GET,[](){server.send(200,"application/json",statusJson());});
  server.on("/send",HTTP_POST,[](){
    if(!server.hasArg("text")||server.arg("text").length()==0){server.send(400,"text/plain","Type a message first");return;}
    bool ok=sendMessage(server.arg("text"));server.send(ok?200:500,"text/plain",statusText);
  });
  server.on("/config",HTTP_POST,handleConfig);
  server.on("/reset",HTTP_POST,[](){if(prefsReady)prefs.clear();server.send(200,"text/plain","Reset; rebooting");delay(300);ESP.restart();});
  server.onNotFound([](){server.sendHeader("Location","/");server.send(302,"text/plain","");});server.begin();
  Serial.printf("%s messenger AP %s: %s\n",BOARD_NAME,ssid.c_str(),apOk?"READY":"FAILED");
}

void setup(){
  Serial.begin(115200);delay(120);
  uint64_t mac=ESP.getEfuseMac();nodeId=(uint32_t)(mac^(mac>>32));
  startAP();
  prefs.begin("holden-msg",false);prefsReady=true;loadConfig();
  if(!keyConfigured())statusText="paste the C3 repeater shared key in settings";
}

void loop(){
  server.handleClient();
  serviceReceive();
  static uint32_t retryAt=0;
  if((!radioOk||radioMode!=RADIO_LORA_RX)&&millis()>1200&&(uint32_t)(millis()-retryAt)>5000){retryAt=millis();startLoRaRx();}
  delay(1);
}
