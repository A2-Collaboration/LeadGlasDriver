ARDUINO_PORT = $(shell find /dev/serial/by-id/ -name "usb-FTDI_FT232R_USB_UART*")
ARDUINO_LIBS = jeelib

#BOARD_TAG    = mega

MCU          = atmega328p

MONITOR_PORT = $(ARDUINO_PORT)

ARDUINO_DIR  = /usr/share/arduino

include /usr/share/arduino/Arduino.mk
