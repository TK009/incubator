
SKETCH=incubator.ino

#mkfile_path := $(abspath $(lastword $(MAKEFILE_LIST)))
#current_dir := $(patsubst %/,%,$(dir $(mkfile_path)))
#ESP_ROOT=$(HOME)/Arduino/esp8266

LIBS += $(ESP_LIBS)/Servo \
	$(ESP_LIBS)/Wire \
	$(ESP_LIBS)/ESP8266WiFi \
	$(ESP_LIBS)/Ticker \
	ESPAsyncWebServer \
	ESPAsyncTCP \
	DFRobot_SHT20 \
	Arduino-PID-Library \
	.
	#esp8266-SNTPClock \
	#$(ESP_LIBS)/Ticker \

UPLOAD_PORT = /dev/ttyUSB0
BOARD=nodemcuv2
#nodemcu #nodemcuv2

# Didn't like /tmp ???
# BUILD_DIR=bin/
FLASH_DEF=4M2M
FS_DIR=data
# UPLOAD_SPEED=460800
#
# EXTRAFLAGS=-DDEBUG_ESP_PORT=Serial
#
include $(HOME)/coding/hacks/makeEspArduino/makeEspArduino.mk
