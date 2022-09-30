#TARGET?=Mega2560
TARGET?=ESP32
#TARGET=ESP32S3
ifeq ("$(TARGET)", "ESP32S3")
PORT?=/dev/ttyACM0
ESP32_PSRAM=opi
ESP32_FLASHSIZE=16MB
ESP32S3_CDCONBOOT=cdc
ESP32_PARTFILE=partitions_ESP32S3.csv
ESP32_PARTSCHEME=app3M_fat9M_16MB
else
PORT?=/dev/ttyUSB0
ESP32_FLASHSIZE=16M
ESP32_PARTSCHEME=min_spiffs
endif

ESP32_FILESYSTEM=spiffs
ESP32_FILESYSTEM_PART=spiffs

#ESP32_DEBUGLEVEL=verbose
GITHUB_REPOS= \
reeltwo/Reeltwo \
adafruit/Adafruit_NeoPixel \
adafruit/Adafruit-GFX-Library \
adafruit/Adafruit_SSD1306 \
adafruit/Adafruit_BusIO \
rimim/espsoftwareserial \
reeltwo/PCF8574

ifeq ("$(TARGET)", "ESP32S3")
ARDUINO_OPTS+='-prefs="compiler.cpp.extra_flags=-DROAM_A_DOME_DISPLAY=1"'
else ifeq ("$(FULLSIZE)","1")
ARDUINO_OPTS+='-prefs="compiler.cpp.extra_flags=-DROAM_A_DOME_FULLSIZE_PCB=1"'
else ifeq ("$(LILYGO)","1")
ARDUINO_OPTS+='-prefs="compiler.cpp.extra_flags=-DLILYGO_MINI32=1"'
else
ARDUINO_OPTS+='-prefs="compiler.cpp.extra_flags=-DROAM_A_DOME_COMPACT_PCB=1"'
endif

include ../Arduino.mk
