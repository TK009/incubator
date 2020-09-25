
#include "util.h"


float tempHistory[HistoryLength];
float humHistory[HistoryLength];
float powerHistory[HistoryLength];
int numberOfHours = 0;
int numberOfAverageNmubers = 0;




double readTemperatureNew(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  if(tempC == DEVICE_DISCONNECTED_C) 
  {
    //Serial.println("Error: Could not read temperature data");
    return 255;
  }
  //Serial.print("Temp C: ");
  //Serial.println(tempC);
  
  return tempC;
}

double readTemperatures() {
    sht1comms.begin(Sht1sdaPin,Sht1sdcPin);
    double temp1 = sensor1.readTemperature();
    //sht2comms.begin(Sht2sdaPin,Sht2sdcPin);
    double temp2 = 255;//sensor2.readTemperature();
    //sprint(temp1); sprint("C "); sprint(temp2); sprint("C ");

    if (temp1 < 130 && temp2 < 130) return (temp1 + temp2) * 0.5;
    if (temp1 < 130) return temp1;
    if (temp2 < 130) return temp2;
    else sfprintln("ERROR: Both temperature readings invalid!");
    return 9001;
}

double readHumidities() {
    sht1comms.begin(Sht1sdaPin,Sht1sdcPin);
    double temp1 = sensor1.readHumidity();
    //sht2comms.begin(Sht2sdaPin,Sht2sdcPin);
    double temp2 = 255;//sensor2.readHumidity();
    //sprint(temp1); sprint("% "); sprint(temp2); sprint("% ");

    if (temp1 < 100 && temp2 < 100) return (temp1 + temp2) * 0.5;
    if (temp1 < 100) return temp1;
    if (temp2 < 100) return temp2;
    else sfprintln("ERROR: Both humidity readings invalid!");
    return 0;
}






PID heaterController(
    &TemperatureInput, &HeaterOutput, &s.Setpoint,
    s.Kp, s.Ki, s.Kd, P_ON_E, DIRECT); // proportional on error or measurement P_ON_E or P_ON_M

Ticker HeaterTimer;

void activateMeasurement() {
    shouldMeasure = true;
}
void initHeater() {
    sfprintln("initHeater");

    //pinMode(HeaterPin, PWM);
    analogWrite(HeaterPin, 1);

    heaterController.SetOutputLimits(0.0, s.MaxHeater);
    heaterController.SetMode(AUTOMATIC);
    heaterController.SetTunings(s.Kp,s.Ki,s.Kd);
    HeaterTimer.attach(1, activateMeasurement);
    ok();
}

void updateHeater() {
    // read sensors  
    
    double TempSum = 0;
    int Counter = 0;
    
    Temp0 = readTemperatures();
    if (Temp0 < 81) {
      TempSum += Temp0;
      Counter++;
    }
    //else { sfprintln("Invalid input on the big thermo/humidity meter"); }
    
    Temp1 = readTemperatureNew(outsideThermometer);
    if (Temp1 < 81){
      TempSum += Temp1;
      Counter++;
    }
    //else { sfprintln("Invalid input on thermometer 1"); }

    Temp2 = readTemperatureNew(insideThermometer);
    if (Temp2 < 81) {
      TempSum += Temp2;
      Counter++;
    }
    //else { sfprintln("Invalid input on thermometer 2"); }

    TemperatureInput = TempSum/Counter;
    Humidity = readHumidities();
    sensors.requestTemperatures();

    // update heater
    heaterController.Compute();
    analogWrite(HeaterPin, (int) (HeaterOutput * PWMRANGE));
    sfprint("Temp0 "); sprint(Temp0); sfprint("C, Temp1 "); sprint(Temp1); sfprint("C, Temp2 "); sprint(Temp2);
    sfprint("C Humi: "); sprint(Humidity); sfprint("Input: "); sprint(TemperatureInput); sfprint("C Heatpower: ");
    sprintln(HeaterOutput);
    shouldMeasure = false;
}






void initTurner() {
    sfprintln("initPins");

    eggTurner.attach(ServoTurnPin, 750, 2250);//1050, 1950); // Model: SG90 min 1ms, max 2ms
    activateStartingOrientation();

    ok();
}





void turnEggs() {
    if (turnedRight) turnLeft();
    else turnRight();

    turnedRight = !turnedRight;
    shouldTurn = false;
}

void activateTurn() {
    shouldTurn = true;
}

Ticker TurnTimer;

void initTurnScheduler() {
    sfprintln("initTurnScheduler");
    sprint(s.TurnTime); sfprintln(" sec turn interval");
    TurnTimer.attach(s.TurnTime, activateTurn);
    ok();
}






#define arg(s) request->arg(F(s))
#define has(s) request->hasArg(F(s))

void saveHandler(AsyncWebServerRequest *request){
    if (has("setpoint")) {
        s.Setpoint = arg("setpoint").toFloat();
        sfprint("Received setpoint: "); sprintln(s.Setpoint);
    }
    if (has("maxpower")) {
        s.MaxHeater = min(1.0, (double) arg("maxpower").toFloat());
        sfprint("Received maxpower: "); sprintln(s.MaxHeater);
    }
    if (has("kp") && has("ki") && has("kd")) {
        s.Kp = arg("kp").toFloat();
        s.Ki = arg("ki").toFloat();
        s.Kd = arg("kd").toFloat();
        sfprintln("Received heater controller tunings: "); sprintln(s.Kp); sprintln(s.Ki); sprintln(s.Kd);
    }
    if (has("turnangle")) {
        s.TurnDegrees = arg("turnangle").toInt();
        sfprint("Received Turn angle: "); sprintln(s.TurnDegrees);
    }
    if (has("turntime")) {
        s.TurnTime = arg("turntime").toFloat()*60;
        initTurnScheduler();
    }

    initHeater();

    EEPROM.put(0, s);
    EEPROM.commit();

    request->send(200, "text/plain", "saved");
}

void fetchHandler(AsyncWebServerRequest *request){
    
    //server.send(200, "application/json", );
    auto stream = request->beginResponseStream("application/json");
    stream->printf(
        ("{\"setpoint\": %f, \"maxpower\": %f, \"kp\": %f, \"ki\": %f, \"kd\": %f,"
        " \"turnangle\": %i, \"turntime\": %f, \"temp\": %f, \"humi\": %f, \"power\": %f,"
        "\"tempNew1\" : %f, \"tempNew2\" : %f, \"tempOld\": %f}"),
        s.Setpoint, s.MaxHeater, heaterController.GetKp(), heaterController.GetKi(), heaterController.GetKd(),
        s.TurnDegrees, s.TurnTime/60, TemperatureInput, Humidity, HeaterOutput, Temp1, Temp2,
        Temp0
    );
    request->send(stream);
}
void historyHandler(AsyncWebServerRequest *request){
    
    //server.send(200, "application/json", );
    auto stream = request->beginResponseStream("application/json");
    stream->printf("");
    request->send(stream);
}





void setup() {
    Serial.begin(115200); delay(3000); sfprintln("INIT");

    pinMode(BUILTIN_LED, OUTPUT); ledOn();

    initSettings();

    initWifi("syysmyrsky", "salamamurmeli");

    initSensors();

    initHeater();

    initTurner();

    initTurnScheduler();

    initServer();

    sfprintln("INIT DONE");
	ledOff();
}



void loop() {
    // Just wifi requires a call to loop
    jw.loop(); delay(3);

    // run turner if triggered
    if (shouldTurn) turnEggs();

    if (shouldMeasure) updateHeater();

    ArduinoOTA.handle();
}












