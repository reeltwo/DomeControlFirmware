#TARGET?=Mega2560
TARGET?=ESP32
PORT?=/dev/ttyUSB0
#ESP32_FILESYSTEM=littlefs
#ESP32_PSRAM=enabled
ESP32_FILESYSTEM=spiffs
ESP32_FILESYSTEM_PART=spiffs
ESP32_PARTSCHEME=min_spiffs
ESP32_FLASHSIZE=16M
GITHUB_REPOS= \
reeltwo/Reeltwo \
adafruit/Adafruit_NeoPixel \
adafruit/Adafruit-GFX-Library \
adafruit/Adafruit_SSD1306 \
adafruit/Adafruit_BusIO \
rimim/espsoftwareserial \
xreef/PCF8574_library

include ../Arduino.mk
