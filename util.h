
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include "DFRobot_SHT20.h"
#include <Servo.h>
#include "PID_v1.h"
#include <Ticker.h>
#include "ESPAsyncWebServer.h"
#include "SPIFFSEditor.h"
#include <FS.h>


// DEFAULT SETTINGS CONSTANTS

// full turn deglees 1..180
#define DefaultTurnDegrees 180
#define DefaultSetpoint 37.5
// Settings for 5V heater
#define DefaultKp 5.0
#define DefaultKi 0.08
#define DefaultKd 0.0
// Settings for 12V heater
//#define DefaultKp 2.0
//#define DefaultKi 0.08
//#define DefaultKd 0.0
#define DefaultTurnTime 3 * 60 * 60 //seconds

// CONSTANTS
const char* http_username = "admin";
const char* http_password = "muusi";


#define HeaterPin       D1
#define Sht1sdcPin      D4
#define Sht1sdaPin      D3
#define Sht2sdcPin      D6
#define Sht2sdaPin      D5
#define ServoTurnPin    D2

// these are defaults; best accuracy, no heater and otp reload disabled
#define SHT_ExpectedRegister \
    (USER_REGISTER_RESOLUTION_RH12_TEMP14 | \
    USER_REGISTER_DISABLE_OTP_RELOAD)


// MACROS

#define ledOn()     digitalWrite(LED_BUILTIN, LOW )
#define ledOff()    digitalWrite(LED_BUILTIN, HIGH)
#define turnLeftCmd()  eggTurner.write(90-TurnDegrees/2)
#define turnRightCmd() eggTurner.write(90+TurnDegrees/2)

#define println(s)  Serial.println(s)
#define print(s)    Serial.print(s)
#define fprintln(s) println(F(s))
#define fprint(s)   print(F(s))

void ok()   { fprintln("OK"); }
void fail() { fprintln("FAIL"); }


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
    turnLeftCmd();
    delay(100);
    turnLeftCmd();
    //delay(500);
    //turnLeftCmd();
}

void turnRight() {
    turnRightCmd();
    delay(100);
    turnRightCmd();
    //delay(500);
    //turnRightCmd();
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


void initWifi(const char* ssid, const char* pass) {
    fprintln("initWifi");
    WiFi.hostname("incubator");
    WiFi.begin(ssid, pass);
    fprint("Connecting to "); print(ssid);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500); print(".");
    }
    println();
    fprint("Connected, IP: "); println(WiFi.localIP());

    fprintln("initWifi .. OK");
}

void fetchHandler(AsyncWebServerRequest *request);
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
    server.on("/save", HTTP_POST, saveHandler);
    server.begin();
    ok();
}

