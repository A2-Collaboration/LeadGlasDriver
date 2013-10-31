#include <Ports.h>
#include <PortsLCD.h>
#include <RF12.h>

#define NUM_LCD_COLS 20
#define NUM_LCD_ROWS 4

// register devices
PortI2C I2C1(1);
LiquidCrystalI2C lcd(I2C1);
PortI2C I2C2(2);
AnalogPlug adc(I2C2,0x68);
Port mdriver(3);

// Process variables
long uvolt = 0;
float setpoint_f;
bool direction = false;
bool power = false;
String EpicsRecord = "";
bool EpicsRecComplete = false;

//STATES
enum { LISTEN , MSTART , MGO , MEND , ERROR } state = LISTEN;
enum { State, MStatus , PVADC , Setpoint } outmesg = PVADC;

//utils
void ReadADC(){
	adc.receive();
	long raw = (long) adc.read(0) << 16;
	raw |= (word) adc.read(0) << 8;
	raw |= adc.read(0);
	//byte status = adc.read(1);
	uvolt = raw;
//	raw *= 1000 / (64);
//	uvolt = raw / 1000000.;
} 
void convert_String(){
	char temp[EpicsRecord.length() + 1];
	EpicsRecord.toCharArray( temp, sizeof(temp));
	setpoint_f = atof(temp);
}
void initLCDline(byte n, byte startpos = 0){
	for ( byte i = startpos ; i < NUM_LCD_COLS ; ++i){
		lcd.setCursor(i,n);
		lcd.print(' ');
	}
	lcd.setCursor(startpos,n);
}

//communication
void sendState(){

	switch (outmesg){

	case State:
		lcd.setCursor(8,0);
		switch (state){
		case LISTEN:
			lcd.print("LISTEN");
			break;
		case ERROR:
			lcd.print("ERROR!");
			break;
		default:
			lcd.print("MOVING");
			break;
		}
		break;

	case Setpoint:
		initLCDline(2,8);
		lcd.print(setpoint_f);
		break;

	case PVADC:
		initLCDline(1,8);
		lcd.print(uvolt);
		break;

	case MStatus:
		initLCDline(3,8);
		if ( power ){
			if ( direction ) {
				lcd.print("forward");
			} else {
				lcd.print("reverse");
			}
		} else {
			lcd.print("off");
		}
		break;
	}
}
void serialEvent(){
	while (Serial.available()){
		char incoming = (char) Serial.read();
		EpicsRecord += incoming;
		EpicsRecComplete = ( incoming == '\n' );
	}
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
		outmesg = State;
		sendState();
		if ( EpicsRecComplete ){
			outmesg = Setpoint;
			sendState();
			convert_String();
			EpicsRecord = "";
			EpicsRecComplete = false;
			state=MSTART;
		}
		break;

	case MSTART:
		ReadADC();
		outmesg = PVADC;
		sendState();



		outmesg = State;
		sendState();

		state = MGO;
		break;
		
	case MGO:
		outmesg = State;
		sendState();

		state = MEND;
		break;

	case MEND:
		outmesg = State;
		sendState();

		state = LISTEN;
		break;
	}

}
	//
	//power = !power;
	//if ( !power ) direction = !direction;
	//delay(500);
	//
