
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include "DFRobot_SHT20.h"
#include <Servo.h>
#include "PID_v1.h"
#include <Ticker.h>
#include "ESPAsyncWebServer.h"
#include "SPIFFSEditor.h"
#include <FS.h>
#include "JustWifi.h"
#include <ArduinoOTA.h>
#include <EEPROM.h>


// DEFAULT SETTINGS CONSTANTS

// full turn deglees 1..180
#define DefaultTurnDegrees 180
#define DefaultSetpoint 36.1
// Settings for 5V heater
#define DefaultKp 2.0
#define DefaultKi 0.08
#define DefaultKd 0.04
// Settings for 12V heater
//#define DefaultKp 2.0
//#define DefaultKi 0.08
//#define DefaultKd 0.0
#define DefaultTurnTime 3 * 60 * 60 //seconds




// CONSTANTS
const char* http_username = "admin";
const char* http_password = "tipumuusi";
const char* hostname = "incubator";

#define HistoryInterval 15 // min
#define HistoryLength (int)(22 * 24 * (60/HistoryInterval))
#define EepromSize sizeof(Settings)
#define ReconnectTime 20000 // ms

// PINS

#define HeaterPin       D1  // MOSFET Gate
#define Sht1sdcPin      D4  // I2C clock line for temperature&humidity sensor 1
#define Sht1sdaPin      D3  // I2C data line for temperature&humidity sensor 1
#define Sht2sdcPin      D6  // I2C clock line for temperature&humidity sensor 2
#define Sht2sdaPin      D5  // I2C data line for temperature&humidity sensor 2
#define ServoTurnPin    D2  // Servo PWM data line

// these are defaults; best accuracy, no heater and otp reload disabled
#define SHT_ExpectedRegister \
    (USER_REGISTER_RESOLUTION_RH12_TEMP14 | \
    USER_REGISTER_DISABLE_OTP_RELOAD)

//#define StartingOrientation turnRight
#define StartingOrientation false
#define TurnLeftAngle   (90-s.TurnDegrees/2)
#define TurnRightAngle  (90+s.TurnDegrees/2)

// MACROS

#define ledOn()     digitalWrite(LED_BUILTIN, LOW )
#define ledOff()    digitalWrite(LED_BUILTIN, HIGH)

#define sprintln(s)  Serial.println(s)
#define sprint(s)    Serial.print(s)
#define sfprintln(s) Serial.println(F(s))
#define sfprint(s)   Serial.print(F(s))
#define sfprintf(args...)   Serial.printf(args)
#define fprintln(s) println(F(s))
#define fprint(s)   print(F(s))
// TODO ^

void ok()   { sfprintln("OK"); }
void fail() { sfprintln("FAIL"); }
void turnEggs();


// GLOBALS

DFRobot_SHT20 sensor1;
TwoWire sht1comms;
DFRobot_SHT20 sensor2;
TwoWire sht2comms;

Servo eggTurner;

bool heating = false,
     turnedRight = false,
     shouldTurn = false,
     shouldMeasure = true;
double Humidity,
       TemperatureInput,
       HeaterOutput;

typedef struct Settings {
    double Setpoint = DefaultSetpoint;
    double MaxHeater = 1.0;
    double Kp = DefaultKp;
    double Ki = DefaultKi;
    double Kd = DefaultKd;
    double TurnTime = DefaultTurnTime;
    int TurnDegrees = DefaultTurnDegrees;
} Settings;

Settings s;


// FUNCTIONS

void activateTurn();

void activateStartingOrientation() {
  turnedRight = StartingOrientation;
  activateTurn();
}

void turnLeft() {
    for (int angle=eggTurner.read(); angle>TurnLeftAngle; angle=angle-1){
        eggTurner.write(angle);
        delay(50);
    }
}


void turnRight() {
    for (int angle=eggTurner.read(); angle<TurnRightAngle; angle=angle+1){
        eggTurner.write(angle);
        delay(50);
    }
}


// return true on error
bool testSensor(DFRobot_SHT20 sensor) {

    // check that register has default values (best accuracy and no heater
    byte reg = sensor.readUserRegister();
    if (reg != SHT_ExpectedRegister) {
        sfprint("WARN: Unexpected register value: "); sprintln(reg);

        // try to write the default settings
        sensor.writeUserRegister(SHT_ExpectedRegister);
        reg = sensor.readUserRegister();
        if (reg != SHT_ExpectedRegister) {
            sfprint("ERROR: sensor not working, register: "); sprintln(reg);
            return true;
        }
    }
    
    float temp = sensor.readTemperature();

    sfprint("temperature: "); sprintln(temp);
    if (temp > 130) {
        sfprintln("ERROR: invalid temperature");
        return true;
    }

    float humi = sensor.readHumidity();

    sfprint("humidity: "); sprintln(humi);
    if (humi > 100) {
        sfprint("ERROR: invalid humidity");
        return true;
    }

    return false;
}



void initSettings() {
    sfprintln("initSettings");

    EEPROM.begin(EepromSize);
    EEPROM.get(0, s);

    ok();
}



ADC_MODE(ADC_VCC); // Measure power supply for debugging

void initSensors() {
    sfprintln("initSensors");

    bool error = false;

    sfprint("Testing sensor1 on pin "); sprintln(Sht1sdaPin);
    sht1comms.begin(Sht1sdaPin,Sht1sdcPin);
    sensor1.initSHT20(sht1comms);
    error |= testSensor(sensor1);

    sfprint("Testing sensor2 on pin "); sprintln(Sht2sdaPin);
    sht2comms.begin(Sht2sdaPin,Sht2sdcPin);
    sensor2.initSHT20(sht2comms);
    error |= testSensor(sensor2);

    sfprint("initSensors .. ");
    if (error) fail();
    else ok();
}




void infoWifi() {
    if (WiFi.isConnected()) {
        uint8_t * bssid = WiFi.BSSID();

        sfprint("[WIFI] MODE STA -------------------------------------\n");
        sfprintf("[WIFI] SSID  "); sprintln(WiFi.SSID());
        sfprintf("[WIFI] BSSID %02X:%02X:%02X:%02X:%02X:%02X\n",
            bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]
        );
        sfprintf("[WIFI] CH    %d\n", WiFi.channel());
        sfprintf("[WIFI] RSSI  %d\n", WiFi.RSSI());
        sfprintf("[WIFI] IP    %s\n", WiFi.localIP().toString().c_str());
        sfprintf("[WIFI] MAC   %s\n", WiFi.macAddress().c_str());
        sfprintf("[WIFI] GW    %s\n", WiFi.gatewayIP().toString().c_str());
        sfprintf("[WIFI] MASK  %s\n", WiFi.subnetMask().toString().c_str());
        sfprintf("[WIFI] DNS   %s\n", WiFi.dnsIP().toString().c_str());
        #if defined(ARDUINO_ARCH_ESP32)
            sfprintf("[WIFI] HOST  %s\n", WiFi.getHostname());
        #else
            sfprintf("[WIFI] HOST  %s\n", WiFi.hostname().c_str());
        #endif
        sfprint("[WIFI] ----------------------------------------------\n");
    }

    if (WiFi.getMode() & WIFI_AP) {
        sfprint("[WIFI] MODE AP --------------------------------------\n");
        sfprintf("[WIFI] SSID  %s\n", jw.getAPSSID().c_str());
        sfprintf("[WIFI] IP    %s\n", WiFi.softAPIP().toString().c_str());
        sfprintf("[WIFI] MAC   %s\n", WiFi.softAPmacAddress().c_str());
        sfprint("[WIFI] ----------------------------------------------\n");
    }
}


void infoCallback(justwifi_messages_t code, char * parameter) {
    // -------------------------------------------------------------------------
    if (code == MESSAGE_TURNING_OFF) {
        sfprint("[WIFI] Turning OFF\n");
    }
    if (code == MESSAGE_TURNING_ON) {
        sfprint("[WIFI] Turning ON\n");
    }
    // -------------------------------------------------------------------------
    if (code == MESSAGE_SCANNING) {
        sfprint("[WIFI] Scanning\n");
    }
    if (code == MESSAGE_SCAN_FAILED) {
        sfprint("[WIFI] Scan failed\n");
    }
    if (code == MESSAGE_NO_NETWORKS) {
        sfprint("[WIFI] No networks found\n");
    }
    if (code == MESSAGE_NO_KNOWN_NETWORKS) {
        sfprint("[WIFI] No known networks found\n");
    }
    if (code == MESSAGE_FOUND_NETWORK) {
        sfprintf("[WIFI] %s\n", parameter);
    }
    // -------------------------------------------------------------------------
    if (code == MESSAGE_CONNECTING) {
        sfprintf("[WIFI] Connecting to %s\n", parameter);
    }
    if (code == MESSAGE_CONNECT_WAITING) {
        // too much noise
    }
    if (code == MESSAGE_CONNECT_FAILED) {
        sfprintf("[WIFI] Could not connect to %s\n", parameter);
    }
    if (code == MESSAGE_CONNECTED) {
        infoWifi();
    }
    if (code == MESSAGE_DISCONNECTED) {
        sfprint("[WIFI] Disconnected\n");
    }
    // -------------------------------------------------------------------------
    if (code == MESSAGE_ACCESSPOINT_CREATED) {
        infoWifi();
    }
    if (code == MESSAGE_ACCESSPOINT_DESTROYED) {
        sfprint("[WIFI] Disconnecting access point\n");
    }
    if (code == MESSAGE_ACCESSPOINT_CREATING) {
        sfprint("[WIFI] Creating access point\n");
    }
    if (code == MESSAGE_ACCESSPOINT_FAILED) {
        sfprint("[WIFI] Could not create access point\n");
    }
    // ------------------------------------------------------------------------
    if (code == MESSAGE_WPS_START) {
        sfprint("[WIFI] WPS started\n");
    }
    if (code == MESSAGE_WPS_SUCCESS) {
        sfprint("[WIFI] WPS succeded!\n");
    }
    if (code == MESSAGE_WPS_ERROR) {
        sfprint("[WIFI] WPS failed\n");
    }
    // ------------------------------------------------------------------------
    if (code == MESSAGE_SMARTCONFIG_START) {
        sfprint("[WIFI] Smart Config started\n");
    }
    if (code == MESSAGE_SMARTCONFIG_SUCCESS) {
        sfprint("[WIFI] Smart Config succeded!\n");
    }
    if (code == MESSAGE_SMARTCONFIG_ERROR) {
        sfprint("[WIFI] Smart Config failed\n");
    }
};



void initWifi(const char* ssid, const char* pass) {
    sfprintln("initWifi");

    jw.setHostname(hostname);
    jw.setReconnectTimeout(ReconnectTime);

    // Callbacks
    jw.subscribe(infoCallback);

    // AP mode only as fallback
    jw.enableAP(false);
    jw.enableAPFallback(true);

	// Enable STA mode (connecting to a router)
    jw.enableSTA(true);

    // Configure it to scan available networks and connect in order of dBm
    jw.enableScan(true);

    jw.addNetwork(ssid, pass);

    //sfprint("Connecting to "); sprint(ssid);

    //while (WiFi.status() != WL_CONNECTED) {
    //    delay(500); sprint(".");
    //}
    //sprintln();
    //fsprint("Connected, IP: "); sprintln(WiFi.localIP());

	ArduinoOTA.setHostname(hostname);
	ArduinoOTA.setPassword(http_password);
    ArduinoOTA.onStart(activateStartingOrientation); // do not catapult eggs
	ArduinoOTA.begin();

    sfprintln("initWifi .. OK");
}

void fetchHandler(AsyncWebServerRequest *request);
void historyHandler(AsyncWebServerRequest *request);
void saveHandler(AsyncWebServerRequest *request);

void statusHandler(AsyncWebServerRequest *request) {
    auto stream = request->beginResponseStream("application/json");
    stream->fprint("{\"heap\":");
    stream->print(ESP.getFreeHeap());
    stream->fprint(",\"resetReason\":\"");
    stream->print(ESP.getResetReason());
    stream->fprint("\",\"fragmentation\":");
    stream->print(ESP.getHeapFragmentation());
    stream->fprint(",\"maxFreeBlockSize\":");
    stream->print(ESP.getMaxFreeBlockSize());
    stream->fprint(",\"programMD5\":\"");
    stream->print(ESP.getSketchMD5());
    stream->fprint("\",\"vcc\":");
    stream->print(ESP.getVcc());
    stream->fprint("}");
    request->send(stream);
}

AsyncWebServer server(80);

void initServer(){
    sfprintln("initServer");
    SPIFFS.begin();
    server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");
    server.addHandler(new SPIFFSEditor(http_username,http_password));

    server.on("/fetch",        HTTP_GET, fetchHandler);
    server.on("/status",       HTTP_GET, statusHandler);
    server.on("/fetchHistory", HTTP_GET, historyHandler);
    server.on("/save",         HTTP_POST, saveHandler);
    server.on("/turnBoot",     HTTP_POST, [](AsyncWebServerRequest *r){
            activateStartingOrientation(); r->send(200); });
    server.on("/turn",         HTTP_POST, [](AsyncWebServerRequest *r){
            activateTurn(); r->send(200); });
    server.on("/reboot",       HTTP_POST, [](AsyncWebServerRequest *r){
            r->send(200); ESP.restart(); });
    server.begin();
    ok();
}

