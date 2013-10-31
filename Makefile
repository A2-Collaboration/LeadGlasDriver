ARDUINO_PORT = /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A9014YWT-if00-port0
ARDUINO_LIBS = jeelib

MCU          = atmega328p

MONITOR_PORT = /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A9014YWT-if00-port0

ARDUINO_DIR  = /usr/share/arduino

include ../Arduino.mk
