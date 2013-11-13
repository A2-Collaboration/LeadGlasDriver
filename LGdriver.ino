#include <Arduino.h>
#include <Ports.h>
#include <PortsLCD.h>
#include <RF12.h>

#define NUM_LCD_COLS 20
#define NUM_LCD_ROWS 4
// maximum ADC-Voltage
#define UMAX 2.048

// TIMING
#define T_SAMPLE 300 // [ ms ]

// constants
const float lsb_adc = 15.625 / 1000000; 

// register devices
//LCD
PortI2C I2C1(1);  // JEEPORT no 3
LiquidCrystalI2C lcd(I2C1);
//ADC
PortI2C I2C2(3); // JEEPORT no 1
AnalogPlug adc(I2C2,0x68);
// RELAY
Port mdriver(4); // JEEPORT no 4

// Process variables
float adc_f = 0;
bool direction = false;
bool power = false;

//STATES
enum { SEND } state = SEND;
enum { State, PVADC } outmesg = PVADC;

//  utils
void ReadADC(){
	adc.receive();
	long raw = (long) adc.read(0) << 16;
	raw |= (word) adc.read(0) << 8;
	raw |= adc.read(0);
	byte status = adc.read(1);
	adc_f = raw * lsb_adc;
} 

//communication
void sendState(){

	switch (outmesg){

	case State:
		lcd.setCursor(8,0);
		switch (state){
		case SEND:
			lcd.print("Sending Voltage:");
			break;
		default:
			lcd.print("ERROR! UDST");
			break;
		}
		break;

	case PVADC:
		lcd.setCursor(8,1);
		lcd.print(adc_f,4);
		lcd.print("V   ");
		Serial.print("adcpos");
		Serial.println(adc_f,4);
		break;
	default:
		lcd.setCursor(8,1);
		lcd.print("ERROR      ");
		break;
	}
}

//MAIN EVT-LOOPS:
void setup(void) {
	// Init Display
	lcd.begin(NUM_LCD_COLS,NUM_LCD_ROWS);

	//Init serial console
	Serial.begin(57600);

	//Init adc
	adc.begin(0x1C);

	//relais
	mdriver.digiWrite(0);
	mdriver.mode(OUTPUT);
	mdriver.digiWrite2(0);
	mdriver.mode2(OUTPUT);
	
	//persistant lines on lcd:
	lcd.setCursor(0,0);
	lcd.print("Status: ");
	lcd.setCursor(0,1);
	lcd.print("ADC:    ");
	lcd.setCursor(0,2);
	lcd.print("For calibration, std");
	lcd.setCursor(0,3);
	lcd.print("Mode:checkout master");
}


void loop() {

	switch(state){

	case SEND:

		outmesg = State;
		sendState();
		
		delay(T_SAMPLE);
		ReadADC();
		outmesg = PVADC;
		sendState();
		break;
	}

}
