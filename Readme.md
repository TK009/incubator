Software
========

features
--------

* Automatic PID control for heating eggs to specific temperature
* Automatic Servo motor for turning eggs at specific interval
* A web page
   - nice real-time gauges for monitoring humidity, temperature and power of the heater 
   - graph of avarage humidity, temperature and power up to 22 days
   - change settings: temperature set point, PID parameters, turn interval and
     turn angle
   - set turn angle to zero to disable turning
* Double humidity and temperature sensor for reliability

Getting started
---------------

1. `git clone --recursive`
2. Download Arduino esp makefile and fix location in `Makefile`
3. Connect ESP and update usb port and other settings in `Makefile`
4. `make upload`

See the top of `util.h` for default settings. When the device is started it
tries to connect to default wifi network, but if not found, it should create a
new wifi access point for setting it up (but I have not tested it yet).

Hardware
========

* ESP8266 NodeMCU v2 development board (any ESP8266 module with enough pins will do)
* 2pcs SHT20 temperature and humidity sensor
* cheap hobby servo motor (SG90)
* 10m of thin wire for creating a DC heater (or 5m of two core wire with one end shorted)
* Mosfet (e.g. IRLZ44 or IRLZ44n) for heater
* 10k resistor for the mosfet gate to ground and additionally a small diode to protect voltage spikes
* Heat sink for the mosfet (recommended for safety)
* 3.3V to 5V logic voltage converter module to connect servo and heater mosfet
* PC power supply (Only 5V line is used)
* 12V PC fan (connected to 5V instead of 12V for slower rpm)

Assembly
--------

* Look near top of `util.h` file for esp pins. If your esp module does not have written arduino pins, search for the layout from internet.
* Connect data lines

```
#define HeaterPin       D1  // MOSFET Gate
#define Sht1sdcPin      D4  // I2C clock line for temperature&humidity sensor 1
#define Sht1sdaPin      D3  // I2C data line for temperature&humidity sensor 1
#define Sht2sdcPin      D6  // I2C clock line for temperature&humidity sensor 2
#define Sht2sdaPin      D5  // I2C data line for temperature&humidity sensor 2
#define ServoTurnPin    D2  // Servo PWM data line
// Note: sdc lines could be shared between sensors
```

* ESP8266 handles only 3.3V input/output signals so converter might be needed for some components like servo motor and mosfet.
* Connect voltage lines
* If using a PC power supply: connect the green wire to one of the black lines (e.g. with a paper clip)
* Connect +5V (red) to heater wire, and the other end to mosfet drain, if the wire doesn't heat enough: change to 12V or change the heater wire length or diameter.
* Connect mosfet source to ground and add 10k resistor from gate to ground (drain)

