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
#define T_WAITMOTOR 2500 // [ ms ]    ( wait for stop and adc again )
#define T_SMALLSTEP 150  // [ ms ]  ( In position wackeln... )

// constants
const float lsb_adc = 15.625 / 1000000; 
// Coarse Hysteresis
const float coardse_l = 0.0040;
const float coardse_r = 0.0040;
// Fine Hysteresis
const float fine_l = 0.0002;
const float fine_r = 0.0002;

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
float setpoint_f;
bool direction = false;
bool power = false;
String EpicsRecord = "";
bool EpicsRecComplete = false;

//STATES
enum { LISTEN , MSTART , MGO ,MSTARTFINE, MGOFINE,  MEND , ERROR } state = LISTEN;
enum { State, MStatus , PVADC , Setpoint } outmesg = PVADC;

//  utils
void ReadADC(){
	adc.receive();
	long raw = (long) adc.read(0) << 16;
	raw |= (word) adc.read(0) << 8;
	raw |= adc.read(0);
	byte status = adc.read(1);
	adc_f = raw * lsb_adc;
} 

void convert_String(){
	char temp[EpicsRecord.length() + 1];
	EpicsRecord.toCharArray( temp, sizeof(temp));
	setpoint_f = atof(temp);
}

//communication
void sendState(){

	switch (outmesg){

	case State:
		lcd.setCursor(8,0);
		switch (state){
		case LISTEN:
			lcd.print("LISTEN     ");
			break;
		case ERROR:
			lcd.print("ERROR STATE");
			break;
		case MSTART:
			lcd.print("SET DIR    ");
			break;
		case MSTARTFINE:
			lcd.print("SET FINEDIR");
			break;
		case MGO:
			lcd.print("MOVING     ");
			break;
		case MGOFINE:
			lcd.print("MOVING SLOW");
			break;
		case MEND:
			lcd.print("WAITING    ");
			break;
		default:
			lcd.print("ERROR! UDST");
			break;
		}
		break;

	case Setpoint:
		lcd.setCursor(8,2);
		lcd.print(setpoint_f,4);
		lcd.print("V");
		lcd.print(" ");
		break;

	case PVADC:
		lcd.setCursor(8,1);
		lcd.print(adc_f,4);
		lcd.print("V  ");
		Serial.print("adcpos");
		Serial.println(adc_f,4);
		break;

	case MStatus:
		lcd.setCursor(8,3);
		if ( power ){
			Serial.print("0s");
			Serial.println(1);
			if ( direction ) {
				lcd.print("forward");
				Serial.print("1s");
				Serial.println(1);
			} else {
				lcd.print("reverse");
				Serial.print("1s");
				Serial.println(-1);
			}
		} else {
			Serial.print("0s");
			Serial.println(0);
			lcd.print("off    ");
		}
		break;
	}
}
void serialEvent(){
	//while (Serial.available()){
	while (!EpicsRecComplete){
		char incoming = (char) Serial.read();
		EpicsRecord += incoming;
		EpicsRecComplete = ( incoming == '\n' );
	}
}

// Prepare to MOVE
void Init_Start(){
	convert_String();
	outmesg = Setpoint;
	sendState();
	EpicsRecord = "";
	EpicsRecComplete = false;
	state=MSTART;
}

// Motor - CONTROLS
void PowerOn(){
	power = true;
	mdriver.digiWrite(HIGH);
	outmesg = MStatus;
	sendState();
}
void PowerOff(){
	power = false;
	mdriver.digiWrite(LOW);
	outmesg = MStatus;
	sendState();
}
void _set_to(bool dir){
	if ( direction != dir){	
		// switch off motor befor changing polarity
		bool power_old = power;
		PowerOff();
		delay(500);
		direction = dir;
		if ( dir ){
			mdriver.digiWrite2(HIGH);
		} else {
			mdriver.digiWrite2(LOW);
		}
		sendState();
		delay(500);
		if (power_old) PowerOn();
	}
}
bool SetMotorDirection(){
	if ( ( setpoint_f - coardse_l < adc_f )
	  && ( adc_f < setpoint_f + coardse_r ) ){
		return 0;
	}
	_set_to( adc_f < setpoint_f );
	return 1;
}
bool SetMotorDirectionFine(){
	if ( ( setpoint_f - fine_l < adc_f )
	  && ( adc_f < setpoint_f + fine_r ) ){
		return 0;
	}
	_set_to( adc_f < setpoint_f );
	return 1;
}

//MAIN EVT-LOOPS:
void setup(void) {
	// Init Display
	lcd.begin(NUM_LCD_COLS,NUM_LCD_ROWS);

	//Init serial console
	Serial.begin(57600);
	EpicsRecord.reserve(34);

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
	lcd.print("EPICS:  ");
	lcd.setCursor(0,3);
	lcd.print("Motor:  ");
	
	// init
	ReadADC();
	outmesg = PVADC;
	sendState();
	outmesg = MStatus;
	sendState();
}


void loop() {

	switch(state){

	case LISTEN:
		// Say: Listening to EPICS
		outmesg = State;
		sendState();

		// once msg is finshed, get setpoint and go to MSTART
		if ( EpicsRecComplete ){
			Init_Start();
		}
		break;

	case MSTART:

		// Update ADC - PV
		ReadADC();
		outmesg = PVADC;
		sendState();

		// Determine Direction
		if ( SetMotorDirection() ){
			state = MGO;
		} else {
			state = MSTARTFINE; // In coarse position => finetune
		}
		break;
		
	case MGO:
		outmesg = State;
		sendState();
		
		PowerOn();
		delay(T_SAMPLE);

		state = MSTART;  // test if coarse position is reached or even overcome

		break;

	case MSTARTFINE:


		ReadADC();
		outmesg = PVADC;
		sendState();

		if ( SetMotorDirectionFine() ){
			state = MGOFINE;
		} else {
			state = MEND;
		}
		break;

	case MGOFINE:
		outmesg = State;
		sendState();

		PowerOn();
		delay(T_SMALLSTEP);
		PowerOff();
		delay(T_WAITMOTOR);
		
		state = MSTARTFINE;

		break;

	case MEND:
		outmesg = State;
		sendState();

		PowerOff();
		delay(T_WAITMOTOR);
		ReadADC();
		outmesg = PVADC;
		sendState();

		state = LISTEN;
		break;

	case ERROR:
		outmesg = State;
		sendState();
		PowerOff();
		state = ERROR;
		break;
	}

}
