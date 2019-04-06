
#include "util.h"


bool turnedRight = false, shouldTurn = false, shouldMeasure = true;
double Setpoint = DefaultSetpoint, TemperatureInput, HeaterOutput, Humidity,
    MaxHeater=1.0, TurnTime=DefaultTurnTime;
int TurnDegrees=DefaultTurnDegrees;
float tempHistory[historyLength];
float humHistory[historyLength];
float powerHistory[historyLength];
int numberOfHours = 0;
int numberOfAverageNmubers = 0;





double readTemperatures() {
    sht1comms.begin(Sht1sdaPin,Sht1sdcPin);
    double temp1 = sensor1.readTemperature();
    sht2comms.begin(Sht2sdaPin,Sht2sdcPin);
    double temp2 = sensor2.readTemperature();
    print(temp1); print("C "); print(temp2); print("C ");

    if (temp1 < 130 && temp2 < 130) return (temp1 + temp2) * 0.5;
    if (temp1 < 130) return temp1;
    if (temp2 < 130) return temp2;
    else fprintln("ERROR: Both temperature readings invalid!");
    return 9001;
}

double readHumidities() {
    sht1comms.begin(Sht1sdaPin,Sht1sdcPin);
    double temp1 = sensor1.readHumidity();
    sht2comms.begin(Sht2sdaPin,Sht2sdcPin);
    double temp2 = sensor2.readHumidity();
    print(temp1); print("% "); print(temp2); print("% ");

    if (temp1 < 100 && temp2 < 100) return (temp1 + temp2) * 0.5;
    if (temp1 < 100) return temp1;
    if (temp2 < 100) return temp2;
    else fprintln("ERROR: Both humidity readings invalid!");
    return 0;
}






PID heaterController(
    &TemperatureInput, &HeaterOutput, &Setpoint,
    DefaultKp, DefaultKi, DefaultKd, P_ON_E, DIRECT); // proportional on error or measurement P_ON_E or P_ON_M

Ticker HeaterTimer;

void activateMeasurement() {
    shouldMeasure = true;
}
void initHeater() {
    fprintln("initHeater");

    //pinMode(HeaterPin, PWM);
    analogWrite(HeaterPin, 1);

    heaterController.SetOutputLimits(0.0, MaxHeater);
    heaterController.SetMode(AUTOMATIC);
    HeaterTimer.attach(1, activateMeasurement);
    ok();
}

void updateHeater() {
    // read sensors
    TemperatureInput = readTemperatures();
    Humidity = readHumidities();

    // update heater
    heaterController.Compute();
    analogWrite(HeaterPin, (int) (HeaterOutput * PWMRANGE));
    fprint("Input: "); print(TemperatureInput); fprint("C Heatpower: ");
    println(HeaterOutput);
    shouldMeasure = false;
}






void initTurner() {
    fprintln("initPins");

    eggTurner.attach(ServoTurnPin, 750, 2250);//1050, 1950); // Model: SG90 min 1ms, max 2ms
    StartingOrientation();

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
    fprintln("initTurnScheduler");
    print(TurnTime); fprintln(" sec turn interval");
    TurnTimer.attach(TurnTime, activateTurn);
    ok();
}






#define arg(s) request->arg(F(s))
#define has(s) request->hasArg(F(s))

void saveHandler(AsyncWebServerRequest *request){
    if (has("setpoint")) {
        Setpoint = arg("setpoint").toFloat();
        fprint("Received setpoint: "); println(Setpoint);
    }
    if (has("maxpower")) {
        MaxHeater = min(1.0, (double) arg("maxpower").toFloat());
        fprint("Received maxpower: "); println(MaxHeater);
    }
    if (has("kp") && has("ki") && has("kd")) {
        heaterController.SetTunings(arg("kp").toFloat(),arg("ki").toFloat(),arg("kd").toFloat());
        fprintln("Received heater controller tunings: "); println(heaterController.GetKp()); println(heaterController.GetKi()); println(heaterController.GetKd());
    }
    if (has("turnangle")) {
        TurnDegrees = arg("turnangle").toInt();
        fprint("Received Turn angle: "); println(TurnDegrees);
    }
    if (has("turntime")) {
        TurnTime = arg("turntime").toFloat()*60;
        initTurnScheduler();
    }

    initHeater();

    request->send(200, "text/plain", "saved");
}

void fetchHandler(AsyncWebServerRequest *request){
    
    //server.send(200, "application/json", );
    auto stream = request->beginResponseStream("application/json");
    stream->printf(
        ("{\"setpoint\": %f, \"maxpower\": %f, \"kp\": %f, \"ki\": %f, \"kd\": %f,"
        " \"turnangle\": %i, \"turntime\": %f, \"temp\": %f, \"humi\": %f, \"power\": %f}"),
        Setpoint, MaxHeater, heaterController.GetKp(), heaterController.GetKi(), heaterController.GetKd(),
        TurnDegrees, TurnTime/60, TemperatureInput, Humidity, HeaterOutput
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
    Serial.begin(115200); delay(3000); fprintln("INIT");

    pinMode(BUILTIN_LED, OUTPUT); ledOn();

    initWifi("syysmyrsky", "salamamurmeli");

    initSensors();

    initHeater();

    initTurner();

    initTurnScheduler();

    initServer();

    fprintln("INIT DONE");
	ledOff();
}



void loop() {
    // Just wifi requires a call to loop
    jw.loop(); delay(2);

    // run turner if triggered
    if (shouldTurn) turnEggs();

    if (shouldMeasure) updateHeater();

    ArduinoOTA.handle();
}
