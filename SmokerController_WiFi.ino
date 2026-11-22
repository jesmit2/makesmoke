/*
  Smoker Controller - ESP32 DevKit (Rev1) - BLE + WiFi setup
  - Pit: MAX6675 thermocouple
  - Meat 1 & Meat 2: Analog thermistors
  - 2x I2C LCDs (0x27 and 0x25)
  - Up/Down buttons
  - Fan control + Buzzer
  - Bluetooth LE for phone app
  - NEW: WiFi network + password set from the phone app over BLE (saved in flash)

  - NEW: over-the-air firmware updates. Once the smoker is on WiFi, open
         http://<its IP>/  (or http://smoker.local/) and upload a compiled .bin.

  Too big for your board? Set ENABLE_OTA to 0 (saves ~100+ KB), or ENABLE_WIFI to 0
  for a Bluetooth-only build (smallest). The app copes with either.

  Arduino IDE: Tools > Partition Scheme > "Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)"
  (BLE + WiFi are too big for the default partition, and OTA needs two app slots,
   so do NOT use "Huge APP". Changing the partition scheme requires one USB upload.)

  To build the .bin for an update: Sketch > Export Compiled Binary, then upload the file
  named SmokerController_WiFi.ino.bin (not the "bootloader" or "partitions" ones).
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "max6675.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ---------------- Build options: turn features off to make the sketch smaller ----------------
#define ENABLE_WIFI 1   // 1 = WiFi setup from the app.  0 = Bluetooth only (smallest; also removes OTA)
#define ENABLE_OTA  1   // 1 = firmware update page over WiFi.  0 = saves roughly 100+ KB
#if !ENABLE_WIFI
  #undef  ENABLE_OTA
  #define ENABLE_OTA 0
#endif

#if ENABLE_WIFI
  #include <WiFi.h>
  #include <Preferences.h>
#endif
#if ENABLE_OTA
  #include <WebServer.h>
  #include <Update.h>
  #include <ESPmDNS.h>
#endif

// ---------------- Pins (ESP32 DevKit) ----------------
const int PIN_UP     = 32;
const int PIN_DOWN   = 33;
const int PIN_FAN    = 25;
const int PIN_BUZZER = 26;

// MAX6675 - Pit
const int SCK_PIN = 18;
const int SO_PIN  = 19;
const int CS_PIT  = 5;

MAX6675 thermoPit(SCK_PIN, CS_PIT, SO_PIN);

// Analog meat probes (ADC1 pins, so they keep working while WiFi is on)
const int PIN_MEAT1 = 34;
const int PIN_MEAT2 = 35;

// LCDs
LiquidCrystal_I2C lcd1(0x27, 16, 2);  // Pit + Setpoint
LiquidCrystal_I2C lcd2(0x25, 16, 2);  // Meat probes

// ---------------- State ----------------
float setpoint     = 200.0;
float meatAlarm1   = 165.0;
float meatAlarm2   = 165.0;
float pitTemp      = 0;
float meat1Temp    = 0;
float meat2Temp    = 0;

bool fanOn = false;
unsigned long fanCycleStart = 0;
const unsigned long FAN_ON_MS  = 5UL * 60UL * 1000UL;  // 5 minutes
const unsigned long FAN_OFF_MS = 2UL * 60UL * 1000UL;  // 2 minutes

// Button debounce
unsigned long lastDebounceUp = 0, lastDebounceDown = 0;
const unsigned long DEBOUNCE_MS = 50;
bool lastUpState = HIGH, lastDownState = HIGH;
bool upPressed = false, downPressed = false;

bool alarm1Active = false, alarm2Active = false;

// ---------------- BLE UUIDs ----------------
#define SERVICE_UUID         "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_PIT_UUID        "beb5483e-36e1-4688-b7f5-ea07361b26a1"
#define CHAR_MEAT1_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26a2"
#define CHAR_MEAT2_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26a3"
#define CHAR_SETPOINT_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a4"
#define CHAR_FAN_UUID        "beb5483e-36e1-4688-b7f5-ea07361b26a5"
#define CHAR_WIFI_CMD_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a6"  // app -> ESP32: SCAN | SET\nssid\npass | FORGET
#define CHAR_WIFI_LIST_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a7"  // ESP32 -> app: "rssi|secure|ssid", then "END|n"
#define CHAR_WIFI_STAT_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a8"  // ESP32 -> app: "STATE|ip|ssid"

BLECharacteristic *pPitChar;
BLECharacteristic *pMeat1Char;
BLECharacteristic *pMeat2Char;
BLECharacteristic *pSetpointChar;
BLECharacteristic *pFanChar;
BLECharacteristic *pWifiCmdChar;
BLECharacteristic *pWifiListChar;
BLECharacteristic *pWifiStatChar;

bool deviceConnected = false;

#if ENABLE_WIFI
// ---------------- WiFi state ----------------
Preferences prefs;

enum WifiState { WS_NONE, WS_CONNECTING, WS_CONNECTED, WS_FAILED, WS_DISCONNECTED };
WifiState wifiState = WS_NONE;

String wifiSsid = "";            // network we are using / trying
String wifiPass = "";
String pendSsid = "";            // request from the app, handled in loop()
String pendPass = "";
volatile bool reqScan    = false;
volatile bool reqConnect = false;
volatile bool reqForget  = false;
bool userAttempt = false;        // true while a connection the user just asked for is in progress
unsigned long wifiConnectStart = 0;
const unsigned long WIFI_TIMEOUT_MS = 20000;
String lastStatSent = "";

#if ENABLE_OTA
// ---------------- Firmware update (OTA) ----------------
const char *FW_VERSION = "1.1.0";        // bump this each time you build a new .bin
const char *OTA_USER   = "smitty";
const char *OTA_PASS   = "changeme";     // <-- CHANGE THIS before flashing; anyone on your WiFi who knows it can reflash the smoker
WebServer server(80);
bool webStarted = false;
void startWebServer();                   // defined below; wifiTask() calls it

const char PAGE_TOP[] = R"rawliteral(<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Smoker Firmware</title><style>
body{font-family:system-ui,sans-serif;background:#F2F7FC;color:#0E1B2C;margin:0;padding:16px}
.c{max-width:480px;margin:0 auto;background:#fff;border:1px solid #D6E2F0;border-radius:16px;padding:18px}
h1{font-size:20px;margin:0 0 4px}p{color:#5B6B80;font-size:14px;line-height:1.4}
input[type=file]{width:100%;margin:12px 0}
button{width:100%;font:inherit;font-weight:600;padding:14px;border:0;border-radius:12px;background:#3C82DC;color:#fff}
button:disabled{opacity:.5}
.bar{height:12px;background:#D6E2F0;border-radius:8px;overflow:hidden;margin-top:14px;display:none}
.bar i{display:block;height:100%;width:0;background:#2B7FE0}
#msg{margin-top:12px;font-weight:600}
</style></head><body><div class="c"><h1>Smoker firmware</h1><p>Current version: <b>)rawliteral";

const char PAGE_BOTTOM[] = R"rawliteral(</b></p>
<p>Choose the compiled <b>.bin</b> file (Arduino IDE: Sketch &gt; Export Compiled Binary). The smoker stops controlling the fan while it updates, about a minute. Stay near it and don't unplug it.</p>
<input type="file" id="f" accept=".bin"><button id="b">Upload &amp; update</button>
<div class="bar" id="bar"><i id="p"></i></div><div id="msg"></div></div>
<script>
const f=document.getElementById('f'),b=document.getElementById('b'),m=document.getElementById('msg');
b.onclick=()=>{
 if(!f.files.length){m.textContent='Pick a .bin file first.';return;}
 const x=new XMLHttpRequest(),d=new FormData();d.append('fw',f.files[0]);
 x.open('POST','/update');
 x.upload.onprogress=e=>{if(e.lengthComputable){document.getElementById('bar').style.display='block';document.getElementById('p').style.width=(e.loaded/e.total*100)+'%';m.textContent='Uploading '+Math.round(e.loaded/e.total*100)+'%';}};
 x.onload=()=>{m.textContent=x.status==200?'Done. The smoker is restarting. Reopen the app in about 20 seconds.':'Update failed: '+x.responseText+'. The smoker is restarting.';b.disabled=false;};
 x.onerror=()=>{m.textContent='Connection lost. If the smoker restarted, the update probably worked.';b.disabled=false;};
 b.disabled=true;x.send(d);
};
</script></body></html>)rawliteral";

#endif  // ENABLE_OTA
#endif  // ENABLE_WIFI

// ---------------- Thermistor settings ----------------
const float SERIES_RESISTOR     = 10000.0;
const float THERMISTOR_NOMINAL  = 10000.0;
const float TEMPERATURE_NOMINAL = 25.0;
const float B_COEFFICIENT       = 3950.0;
const int   ADC_MAX             = 4095;

// ---------------- BLE Callbacks ----------------
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("BLE device connected");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("BLE device disconnected");
    BLEDevice::startAdvertising();   // start advertising again
  }
};

class SetpointCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue();   // Fixed for newer ESP32 BLE library

    if (value.length() > 0) {
      float newSet = value.toFloat();
      if (newSet >= 100.0 && newSet <= 400.0) {
        setpoint = newSet;
        Serial.print("New setpoint from app: ");
        Serial.println(setpoint);
      }
    }
  }
};

#if ENABLE_WIFI
// Commands from the app. Only flags are set here; the real work happens in loop().
class WifiCmdCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String v = pCharacteristic->getValue();

    if (v == "SCAN") {
      reqScan = true;
    } else if (v == "FORGET") {
      reqForget = true;
    } else if (v.startsWith("SET\n")) {
      int a = v.indexOf('\n');          // end of "SET"
      int b = v.indexOf('\n', a + 1);   // end of ssid
      if (b > a) {
        String ssid = v.substring(a + 1, b);
        String pass = v.substring(b + 1);
        if (ssid.length() > 0 && ssid.length() <= 32 && pass.length() <= 63) {
          pendSsid = ssid;
          pendPass = pass;
          reqConnect = true;
        }
      }
    }
  }
};

#endif  // ENABLE_WIFI

// ---------------- Helper Functions ----------------
float cToF(float c) {
  return c * 9.0 / 5.0 + 32.0;
}

float readThermistor(int pin) {
  int raw = 0;
  for (int i = 0; i < 8; i++) {
    raw += analogRead(pin);
    delay(2);
  }
  raw /= 8;

  if (raw < 1) raw = 1;
  if (raw >= ADC_MAX) raw = ADC_MAX - 1;

  float resistance = SERIES_RESISTOR / ((float)ADC_MAX / raw - 1.0);

  float steinhart;
  steinhart = resistance / THERMISTOR_NOMINAL;
  steinhart = log(steinhart);
  steinhart /= B_COEFFICIENT;
  steinhart += 1.0 / (TEMPERATURE_NOMINAL + 273.15);
  steinhart = 1.0 / steinhart;
  steinhart -= 273.15;   // Celsius

  return steinhart;
}

void updateLCDs() {
  static float lastPit = -999, lastSet = -999;
  static float lastM1 = -999, lastM2 = -999;
  static bool lastFan = false;
  static bool lastA1 = false, lastA2 = false;

  // LCD 1 - Pit + Setpoint
  if (pitTemp != lastPit || setpoint != lastSet || fanOn != lastFan) {
    lcd1.setCursor(0, 0);
    lcd1.print("Pit:");
    lcd1.print(pitTemp, 0);
    lcd1.print("F   ");

    lcd1.setCursor(0, 1);
    lcd1.print("Set:");
    lcd1.print(setpoint, 0);
    lcd1.print("F");
    if (fanOn) lcd1.print(" FAN");
    else       lcd1.print("    ");

    lastPit = pitTemp;
    lastSet = setpoint;
    lastFan = fanOn;
  }

  // LCD 2 - Meat probes
  if (meat1Temp != lastM1 || meat2Temp != lastM2 ||
      alarm1Active != lastA1 || alarm2Active != lastA2) {

    lcd2.setCursor(0, 0);
    lcd2.print("M1:");
    lcd2.print(meat1Temp, 0);
    lcd2.print("F");
    if (alarm1Active) lcd2.print(" ALM");
    else              lcd2.print("    ");

    lcd2.setCursor(0, 1);
    lcd2.print("M2:");
    lcd2.print(meat2Temp, 0);
    lcd2.print("F");
    if (alarm2Active) lcd2.print(" ALM");
    else              lcd2.print("    ");

    lastM1 = meat1Temp;
    lastM2 = meat2Temp;
    lastA1 = alarm1Active;
    lastA2 = alarm2Active;
  }
}

void handleButtons() {
  bool upReading   = digitalRead(PIN_UP);
  bool downReading = digitalRead(PIN_DOWN);

  if (upReading != lastUpState) lastDebounceUp = millis();
  if ((millis() - lastDebounceUp) > DEBOUNCE_MS) {
    if (upReading == LOW && !upPressed) {
      setpoint += 10.0;
      if (setpoint > 400) setpoint = 400;
      upPressed = true;
    } else if (upReading == HIGH) {
      upPressed = false;
    }
  }
  lastUpState = upReading;

  if (downReading != lastDownState) lastDebounceDown = millis();
  if ((millis() - lastDebounceDown) > DEBOUNCE_MS) {
    if (downReading == LOW && !downPressed) {
      setpoint -= 10.0;
      if (setpoint < 100) setpoint = 100;
      downPressed = true;
    } else if (downReading == HIGH) {
      downPressed = false;
    }
  }
  lastDownState = downReading;
}

void controlFan() {
  unsigned long now = millis();

  if (pitTemp >= setpoint) {
    fanOn = false;
    digitalWrite(PIN_FAN, LOW);
    return;
  }

  if (fanOn) {
    if (now - fanCycleStart >= FAN_ON_MS) {
      fanOn = false;
      fanCycleStart = now;
      digitalWrite(PIN_FAN, LOW);
    }
  } else {
    if (now - fanCycleStart >= FAN_OFF_MS) {
      fanOn = true;
      fanCycleStart = now;
      digitalWrite(PIN_FAN, HIGH);
    }
  }
}

void checkAlarms() {
  alarm1Active = (meat1Temp >= meatAlarm1);
  alarm2Active = (meat2Temp >= meatAlarm2);
  digitalWrite(PIN_BUZZER, (alarm1Active || alarm2Active) ? HIGH : LOW);
}

void updateBLE() {
  if (!deviceConnected) return;

  char buf[16];

  dtostrf(pitTemp, 5, 1, buf);
  pPitChar->setValue(buf);
  pPitChar->notify();

  dtostrf(meat1Temp, 5, 1, buf);
  pMeat1Char->setValue(buf);
  pMeat1Char->notify();

  dtostrf(meat2Temp, 5, 1, buf);
  pMeat2Char->setValue(buf);
  pMeat2Char->notify();

  dtostrf(setpoint, 5, 1, buf);
  pSetpointChar->setValue(buf);

  pFanChar->setValue(fanOn ? "ON" : "OFF");
  pFanChar->notify();
}

#if ENABLE_WIFI
// ---------------- WiFi helpers ----------------
void startWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);          // we store credentials ourselves
  WiFi.setAutoReconnect(true);
  WiFi.disconnect();
  if (wifiPass.length() > 0) WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
  else                       WiFi.begin(wifiSsid.c_str());   // open network
  wifiState = WS_CONNECTING;
  wifiConnectStart = millis();
  Serial.print("WiFi connecting to ");
  Serial.println(wifiSsid);
}

void saveWifiCredentials() {
  prefs.begin("wifi", false);
  prefs.putString("ssid", wifiSsid);
  prefs.putString("pass", wifiPass);
  prefs.end();
}

void clearWifiCredentials() {
  prefs.begin("wifi", false);
  prefs.clear();
  prefs.end();
}

void notifyWifiList(const String &s) {
  if (!deviceConnected) return;
  pWifiListChar->setValue((uint8_t *)s.c_str(), s.length());
  pWifiListChar->notify();
  delay(40);                       // give the BLE stack room between notifications
}

void doWifiScan() {
  if (wifiState == WS_CONNECTING) {
    notifyWifiList("ERR|busy");
    return;
  }
  WiFi.mode(WIFI_STA);
  int n = WiFi.scanNetworks(false, false);   // blocking; hidden networks skipped
  if (n < 0) {
    notifyWifiList("ERR|scan");
    return;
  }
  int sent = 0;
  for (int i = 0; i < n && sent < 16; i++) {
    String ss = WiFi.SSID(i);
    if (ss.length() == 0) continue;
    String line = String(WiFi.RSSI(i)) + "|" +
                  (WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "0" : "1") + "|" + ss;
    notifyWifiList(line);
    sent++;
  }
  WiFi.scanDelete();
  notifyWifiList("END|" + String(sent));
}

void updateWifiStatus() {
  String s;
  switch (wifiState) {
    case WS_NONE:         s = "NONE||"; break;
    case WS_CONNECTING:   s = "CONNECTING||" + wifiSsid; break;
    case WS_CONNECTED:    s = "CONNECTED|" + WiFi.localIP().toString() + "|" + wifiSsid; break;
    case WS_FAILED:       s = "FAILED||" + wifiSsid; break;
    case WS_DISCONNECTED: s = "DISCONNECTED||" + wifiSsid; break;
  }
  if (s == lastStatSent) return;
  lastStatSent = s;
  pWifiStatChar->setValue((uint8_t *)s.c_str(), s.length());   // readable any time
  if (deviceConnected) pWifiStatChar->notify();
  Serial.println("WiFi status: " + s);
}

void wifiTask() {
  if (reqForget) {
    reqForget = false;
    WiFi.disconnect(true, true);
    clearWifiCredentials();
    wifiSsid = "";
    wifiPass = "";
    userAttempt = false;
    wifiState = WS_NONE;
  }

  if (reqConnect) {
    reqConnect = false;
    wifiSsid = pendSsid;
    wifiPass = pendPass;
    userAttempt = true;            // credentials are saved only once the connection works
    startWifi();
  }

  if (reqScan) {
    reqScan = false;
    doWifiScan();
  }

  if (wifiState == WS_CONNECTING) {
    if (WiFi.status() == WL_CONNECTED) {
      wifiState = WS_CONNECTED;
      if (userAttempt) {
        saveWifiCredentials();
        userAttempt = false;
      }
    } else if (millis() - wifiConnectStart > WIFI_TIMEOUT_MS) {
      WiFi.disconnect();
      wifiState = userAttempt ? WS_FAILED : WS_DISCONNECTED;   // wrong password vs. saved network not in range
      userAttempt = false;
    }
  } else if (wifiState == WS_CONNECTED && WiFi.status() != WL_CONNECTED) {
    wifiState = WS_DISCONNECTED;
  } else if (wifiState == WS_DISCONNECTED && WiFi.status() == WL_CONNECTED) {
    wifiState = WS_CONNECTED;
  }

#if ENABLE_OTA
  if (wifiState == WS_CONNECTED) startWebServer();
#endif

  updateWifiStatus();
}

#if ENABLE_OTA
// ---------------- Web server for firmware updates ----------------
void handleRoot() {
  if (!server.authenticate(OTA_USER, OTA_PASS)) return server.requestAuthentication();
  server.send(200, "text/html", String(PAGE_TOP) + FW_VERSION + PAGE_BOTTOM);
}

// Called for every chunk of the uploaded file
void handleUpdateUpload() {
  if (!server.authenticate(OTA_USER, OTA_PASS)) return;
  HTTPUpload &up = server.upload();

  if (up.status == UPLOAD_FILE_START) {
    Serial.printf("OTA start: %s\n", up.filename.c_str());
    digitalWrite(PIN_FAN, LOW);          // the control loop is paused during the update
    digitalWrite(PIN_BUZZER, LOW);
    fanOn = false;
    lcd1.clear(); lcd1.print("Updating...");
    lcd2.clear(); lcd2.print("Do not unplug");
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (Update.write(up.buf, up.currentSize) != up.currentSize) Update.printError(Serial);
  } else if (up.status == UPLOAD_FILE_END) {
    if (Update.end(true)) Serial.printf("OTA done: %u bytes\n", up.totalSize);
    else Update.printError(Serial);
  } else if (up.status == UPLOAD_FILE_ABORTED) {
    Update.abort();
  }
}

// Called once the upload is finished
void handleUpdateDone() {
  if (!server.authenticate(OTA_USER, OTA_PASS)) return server.requestAuthentication();
  bool ok = !Update.hasError();
  server.sendHeader("Connection", "close");
  server.send(ok ? 200 : 500, "text/plain", ok ? "OK" : Update.errorString());
  delay(500);
  ESP.restart();                         // restart either way so the LCDs and fan control start clean
}

void startWebServer() {
  if (webStarted) return;
  server.on("/", HTTP_GET, handleRoot);
  server.on("/update", HTTP_POST, handleUpdateDone, handleUpdateUpload);
  server.begin();
  MDNS.begin("smoker");                  // http://smoker.local
  webStarted = true;
  Serial.println("Update page ready at http://" + WiFi.localIP().toString() + "/");
}

#endif  // ENABLE_OTA
#endif  // ENABLE_WIFI

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  Serial.println("Smoker Controller - BLE + WiFi setup");

  pinMode(PIN_UP, INPUT_PULLUP);
  pinMode(PIN_DOWN, INPUT_PULLUP);
  pinMode(PIN_FAN, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_FAN, LOW);
  digitalWrite(PIN_BUZZER, LOW);

  // I2C + LCDs
  Wire.begin(21, 22);          // SDA=21, SCL=22
  lcd1.init();
  lcd1.backlight();
  lcd2.init();
  lcd2.backlight();
  lcd1.print("Smoker Boot...");
  lcd2.print("BLE Ready");

  // ---------- BLE Setup ----------
  BLEDevice::init("SmokerController");
  BLEDevice::setMTU(185);      // room for long network names in scan results
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // 30 handles: the default of 15 only fits the original five characteristics, so the three
  // WiFi ones were silently dropped and the app saw "Not supported by this firmware".
  BLEService *pService = pServer->createService(BLEUUID(SERVICE_UUID), 30, 0);

  pPitChar      = pService->createCharacteristic(CHAR_PIT_UUID,      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pMeat1Char    = pService->createCharacteristic(CHAR_MEAT1_UUID,    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pMeat2Char    = pService->createCharacteristic(CHAR_MEAT2_UUID,    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pSetpointChar = pService->createCharacteristic(CHAR_SETPOINT_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pFanChar      = pService->createCharacteristic(CHAR_FAN_UUID,      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
#if ENABLE_WIFI
  pWifiCmdChar  = pService->createCharacteristic(CHAR_WIFI_CMD_UUID,  BLECharacteristic::PROPERTY_WRITE);
  pWifiListChar = pService->createCharacteristic(CHAR_WIFI_LIST_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pWifiStatChar = pService->createCharacteristic(CHAR_WIFI_STAT_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
#endif

  pSetpointChar->setCallbacks(new SetpointCallbacks());
#if ENABLE_WIFI
  pWifiCmdChar->setCallbacks(new WifiCmdCallbacks());
#endif

  pPitChar->addDescriptor(new BLE2902());
  pMeat1Char->addDescriptor(new BLE2902());
  pMeat2Char->addDescriptor(new BLE2902());
  pFanChar->addDescriptor(new BLE2902());
#if ENABLE_WIFI
  pWifiListChar->addDescriptor(new BLE2902());
  pWifiStatChar->addDescriptor(new BLE2902());
#endif

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("BLE advertising as --> SmokerController");

#if ENABLE_WIFI
  // ---------- WiFi: reconnect to the saved network, if any ----------
  prefs.begin("wifi", false);
  wifiSsid = prefs.getString("ssid", "");
  wifiPass = prefs.getString("pass", "");
  prefs.end();
  if (wifiSsid.length() > 0) {
    userAttempt = false;
    startWifi();
  }
  updateWifiStatus();
#endif

  delay(500);
}

// ---------------- Loop ----------------
void loop() {
  // Read sensors
  pitTemp   = cToF(thermoPit.readCelsius());
  meat1Temp = cToF(readThermistor(PIN_MEAT1));
  meat2Temp = cToF(readThermistor(PIN_MEAT2));

  handleButtons();
  controlFan();
  checkAlarms();
  updateLCDs();
  updateBLE();
#if ENABLE_WIFI
  wifiTask();
#endif

#if ENABLE_OTA
  // Wait ~400 ms, but keep answering web requests so the update page stays responsive
  unsigned long t0 = millis();
  while (millis() - t0 < 400) {
    if (webStarted) server.handleClient();
    delay(5);
  }
#else
  delay(400);
#endif
}

