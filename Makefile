SKETCH = ./nmea2k.ino
UPLOAD_PORT = /dev/ttyUSB0
CHIP = esp32
BOARD = esp32
BUILD_EXTRA_FLAGS = -std=gnu++17
EXCLUDE_DIRS = ./test

include ./makeEspArduino/makeEspArduino.mk
