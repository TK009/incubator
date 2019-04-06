
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


// DEFAULT SETTINGS CONSTANTS

// full turn deglees 1..180
#define DefaultTurnDegrees 180
#define DefaultSetpoint 37.5
// Settings for 5V heater
#define DefaultKp 5.0
#define DefaultKi 0.08
#define DefaultKd 0.01
// Settings for 12V heater
//#define DefaultKp 2.0
//#define DefaultKi 0.08
//#define DefaultKd 0.0
#define DefaultTurnTime 3 * 60 * 60 //seconds
#define historyLength (22 * 24)


// CONSTANTS
const char* http_username = "admin";
const char* http_password = "tipumuusi";
const char* hostname = "incubator";

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

#define StartingOrientation turnRight
#define TurnLeftAngle   (90-TurnDegrees/2)
#define TurnRightAngle  (90+TurnDegrees/2)

// MACROS

#define ledOn()     digitalWrite(LED_BUILTIN, LOW )
#define ledOff()    digitalWrite(LED_BUILTIN, HIGH)

#define println(s)  Serial.println(s)
#define print(s)    Serial.print(s)
#define fprintln(s) println(F(s))
#define fprint(s)   print(F(s))
#define fprintf(args...)   Serial.printf(args)
// TODO ^

void ok()   { fprintln("OK"); }
void fail() { fprintln("FAIL"); }
void turnEggs();


// GLOBALS

DFRobot_SHT20 sensor1;
TwoWire sht1comms;
DFRobot_SHT20 sensor2;
TwoWire sht2comms;

Servo eggTurner;

bool heating = false;
extern int TurnDegrees;


// FUNCTIONS

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
        fprint("WARN: Unexpected register value: "); println(reg);

        // try to write the default settings
        sensor.writeUserRegister(SHT_ExpectedRegister);
        reg = sensor.readUserRegister();
        if (reg != SHT_ExpectedRegister) {
            fprint("ERROR: sensor not working, register: "); println(reg);
            return true;
        }
    }
    
    float temp = sensor.readTemperature();

    fprint("temperature: "); println(temp);
    if (temp > 130) {
        fprintln("ERROR: invalid temperature");
        return true;
    }

    float humi = sensor.readHumidity();

    fprint("humidity: "); println(humi);
    if (humi > 100) {
        fprint("ERROR: invalid humidity");
        return true;
    }

    return false;
}


void initSensors() {
    fprintln("initSensors");
    bool error = false;

    fprint("Testing sensor1 on pin "); println(Sht1sdaPin);
    sht1comms.begin(Sht1sdaPin,Sht1sdcPin);
    sensor1.initSHT20(sht1comms);
    error |= testSensor(sensor1);

    fprint("Testing sensor2 on pin "); println(Sht2sdaPin);
    sht2comms.begin(Sht2sdaPin,Sht2sdcPin);
    sensor2.initSHT20(sht2comms);
    error |= testSensor(sensor2);

    fprint("initSensors .. ");
    if (error) fail();
    else ok();
}




void infoWifi() {
    if (WiFi.isConnected()) {
        uint8_t * bssid = WiFi.BSSID();

        fprint("[WIFI] MODE STA -------------------------------------\n");
        fprintf("[WIFI] SSID  %s\n", WiFi.SSID().c_str());
        fprintf("[WIFI] BSSID %02X:%02X:%02X:%02X:%02X:%02X\n",
            bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]
        );
        fprintf("[WIFI] CH    %d\n", WiFi.channel());
        fprintf("[WIFI] RSSI  %d\n", WiFi.RSSI());
        fprintf("[WIFI] IP    %s\n", WiFi.localIP().toString().c_str());
        fprintf("[WIFI] MAC   %s\n", WiFi.macAddress().c_str());
        fprintf("[WIFI] GW    %s\n", WiFi.gatewayIP().toString().c_str());
        fprintf("[WIFI] MASK  %s\n", WiFi.subnetMask().toString().c_str());
        fprintf("[WIFI] DNS   %s\n", WiFi.dnsIP().toString().c_str());
        #if defined(ARDUINO_ARCH_ESP32)
            fprintf("[WIFI] HOST  %s\n", WiFi.getHostname());
        #else
            fprintf("[WIFI] HOST  %s\n", WiFi.hostname().c_str());
        #endif
        fprint("[WIFI] ----------------------------------------------\n");
    }

    if (WiFi.getMode() & WIFI_AP) {
        fprint("[WIFI] MODE AP --------------------------------------\n");
        fprintf("[WIFI] SSID  %s\n", jw.getAPSSID().c_str());
        fprintf("[WIFI] IP    %s\n", WiFi.softAPIP().toString().c_str());
        fprintf("[WIFI] MAC   %s\n", WiFi.softAPmacAddress().c_str());
        fprint("[WIFI] ----------------------------------------------\n");
    }
}


void infoCallback(justwifi_messages_t code, char * parameter) {
    // -------------------------------------------------------------------------
    if (code == MESSAGE_TURNING_OFF) {
        fprint("[WIFI] Turning OFF\n");
    }
    if (code == MESSAGE_TURNING_ON) {
        fprint("[WIFI] Turning ON\n");
    }
    // -------------------------------------------------------------------------
    if (code == MESSAGE_SCANNING) {
        fprint("[WIFI] Scanning\n");
    }
    if (code == MESSAGE_SCAN_FAILED) {
        fprint("[WIFI] Scan failed\n");
    }
    if (code == MESSAGE_NO_NETWORKS) {
        fprint("[WIFI] No networks found\n");
    }
    if (code == MESSAGE_NO_KNOWN_NETWORKS) {
        fprint("[WIFI] No known networks found\n");
    }
    if (code == MESSAGE_FOUND_NETWORK) {
        fprintf("[WIFI] %s\n", parameter);
    }
    // -------------------------------------------------------------------------
    if (code == MESSAGE_CONNECTING) {
        fprintf("[WIFI] Connecting to %s\n", parameter);
    }
    if (code == MESSAGE_CONNECT_WAITING) {
        // too much noise
    }
    if (code == MESSAGE_CONNECT_FAILED) {
        fprintf("[WIFI] Could not connect to %s\n", parameter);
    }
    if (code == MESSAGE_CONNECTED) {
        infoWifi();
    }
    if (code == MESSAGE_DISCONNECTED) {
        fprint("[WIFI] Disconnected\n");
    }
    // -------------------------------------------------------------------------
    if (code == MESSAGE_ACCESSPOINT_CREATED) {
        infoWifi();
    }
    if (code == MESSAGE_ACCESSPOINT_DESTROYED) {
        fprint("[WIFI] Disconnecting access point\n");
    }
    if (code == MESSAGE_ACCESSPOINT_CREATING) {
        fprint("[WIFI] Creating access point\n");
    }
    if (code == MESSAGE_ACCESSPOINT_FAILED) {
        fprint("[WIFI] Could not create access point\n");
    }
    // ------------------------------------------------------------------------
    if (code == MESSAGE_WPS_START) {
        fprint("[WIFI] WPS started\n");
    }
    if (code == MESSAGE_WPS_SUCCESS) {
        fprint("[WIFI] WPS succeded!\n");
    }
    if (code == MESSAGE_WPS_ERROR) {
        fprint("[WIFI] WPS failed\n");
    }
    // ------------------------------------------------------------------------
    if (code == MESSAGE_SMARTCONFIG_START) {
        fprint("[WIFI] Smart Config started\n");
    }
    if (code == MESSAGE_SMARTCONFIG_SUCCESS) {
        fprint("[WIFI] Smart Config succeded!\n");
    }
    if (code == MESSAGE_SMARTCONFIG_ERROR) {
        fprint("[WIFI] Smart Config failed\n");
    }
};



void initWifi(const char* ssid, const char* pass) {
    fprintln("initWifi");
    jw.setHostname(hostname);
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

    //fprint("Connecting to "); print(ssid);

    //while (WiFi.status() != WL_CONNECTED) {
    //    delay(500); print(".");
    //}
    //println();
    //fprint("Connected, IP: "); println(WiFi.localIP());

	ArduinoOTA.setHostname(hostname);
	ArduinoOTA.setPassword(http_password);
    ArduinoOTA.onStart(StartingOrientation); // do not catapult eggs
	ArduinoOTA.begin();

    fprintln("initWifi .. OK");
}

void fetchHandler(AsyncWebServerRequest *request);
void historyHandler(AsyncWebServerRequest *request);
void saveHandler(AsyncWebServerRequest *request);

AsyncWebServer server(80);

void initServer(){
    fprintln("initServer");
    SPIFFS.begin();
    server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");
    server.addHandler(new SPIFFSEditor(http_username,http_password));
    server.on("/heap", HTTP_GET, [](AsyncWebServerRequest *request){
            request->send(200, "text/plain", String(ESP.getFreeHeap()));
            });

    server.on("/fetch", HTTP_GET, fetchHandler);
    server.on("/fetchHistory", HTTP_GET, historyHandler);
    server.on("/save", HTTP_POST, saveHandler);
    server.on("/turnBoot", HTTP_POST, [](AsyncWebServerRequest *r){
            StartingOrientation();
            r->send(200);
            });
    server.on("/turn", HTTP_POST, [](AsyncWebServerRequest *r){
            turnEggs();
            r->send(200);
            });
    server.begin();
    ok();
}

