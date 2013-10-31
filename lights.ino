
#define TLC_FADE_BUFFER_LENGTH 64
#include "Tlc5940.h"
#include "tlc_fades.h"

#define MESSAGE_BEGIN 123
#define MESSAGE_END 125

#define SNIPPET_SIZE 4
#define NUM_HANDLERS 128



#define HEAT_HANDLER 126

#define SEND_INTERVAL 1000
#define METRIC_COUNT 8

#define BAT_WINDOW_SIZE 4
#define BATTERY1_PIN 1
#define BATTERY2_PIN 2
#define HEAT_PIN 24

#define ADDR_STATE_BATTERY1 0
#define ADDR_STATE_BATTERY2 1
#define ADDR_CHT 2
#define ADDR_RPM 3


typedef void(*handlerFunc)(unsigned int);

typedef struct {
	handlerFunc func;
	unsigned char snippet[SNIPPET_SIZE];
	bool process;
} handler;

handler handlers[NUM_HANDLERS];

bool lockTLC = false;
unsigned long timer;

void setup()
{
  /* Call Tlc.init() to setup the tlc.
     You can optionally pass an initial PWM value (0 - 4095) for all channels.*/
	Serial1.begin(9600);

	//Setup the TLC
	Tlc.init();
	int i = 0;
	//Register 96 light handlers
	for(i; i < TLC_FADE_BUFFER_LENGTH; i++) {
		handlers[i].func = &handleLights;
		handlers[i].process = false;
	}

}



void loop()
{ 
	processHandlers();
	tlc_updateFades();
	collect();
	sendData();
	
}

//From the main loop, execute all the handlers that need to be processed. 
//They need to be processed if they have received data in the time since the last
//time the handlers were processed. 
void processHandlers() {
	int i = 0;
	for(i; i < NUM_HANDLERS; i++) {
		if(handlers[i].process) {
			(*handlers[i].func)(i);
			handlers[i].process = 0;
		}
	}
}



//If there's an interrupt on Serial1, then do the receive call
void serialEvent1() {
	receive();
}


// Snippet is SNIPPET_SIZE unsigned chars, so 4 bytes. 
// First byte is the address of the handler
// The handler can choose how to interpret the data, so the LED handlers
// convention is the following...


// |<-----1 byte_--> <---- 1 byte ---><------1 byte-------><------1 byte----->
// |     <address>  | <fade_duration> | <start_fade_power> | <end_fade_power> |


int bufIndex = 0;
unsigned char snippet[SNIPPET_SIZE];
int message = 0;
void receive() {
	
	int b = 0;
	if (Serial1.available() > 0) {
		// read the incoming byte:
		b = Serial1.read();
		if(b == MESSAGE_BEGIN) {
			//If the byte is the start of the message token, 
			//initialize some memory for the incoming snippet
			memset(snippet, 0, SNIPPET_SIZE);
			message = 1;
			bufIndex = 0;
			onReceive();
		} else if(b == MESSAGE_END) {
			//If the byte is the end of the message token...
			message = 0;
			onReceived();
		} else {
			//If it's not a start or end token, then it's data,
			//so put the byte in the snippet buf. 
			snippet[bufIndex] = b;
			bufIndex++;
			//If the index is big enough, then we're received a full snippet and 
			//we can make a handler destined for the address specified in the snippet
			//
			if(bufIndex%4 == 0) {
				unsigned char address =  (snippet[0]);
				int i = 0;
				for(i; i < SNIPPET_SIZE; i++) {
					handlers[address].snippet[i] = snippet[i];
				}
  				handlers[address].process = true;
  				bufIndex = 0;
			}
		}
		
	}

}

 





//===================== HANDLERS


///Lights
void handleLights(unsigned int address) {
	handler handle = handlers[address];

	//Power can only be 0-255
	unsigned int endPower   = handle.snippet[3];
	unsigned int startPower = handle.snippet[2];
	unsigned int duration   = handle.snippet[1];
	endPower *= 8;
	startPower *= 8;
	duration *= 100;


	uint32_t tm = timer + 100;
	tlc_addFade(address, startPower, endPower, tm, tm + duration);
}



//==================== RECEIVE ACTIONS

//On starting of receive
void onReceive() {
	lockTLC = true;
	timer = millis();
} 

//On done receiving, just a hook
void onReceived() {
	lockTLC = false;
	
}




//==================== SEND STUFF


unsigned long lastSend = 0;
String vals[METRIC_COUNT];

void sendData() {
	Serial.println(vals[ADDR_CHT]);

	if(lastSend == 0 || ((millis() - lastSend) > SEND_INTERVAL)) {
		lastSend = millis();
		String msgStart = String("<");

		int i = 0;
		for(i; i < METRIC_COUNT; i++) {
			msgStart += vals[i];
			msgStart += ",";
		}
		msgStart = String(msgStart + ">");
		Serial1.print(msgStart);
	}
}



//=================== DATA STUFF

void collect() {
	measureBat1();
	measureBat2();
	measureCHT();
	measureRPM();
}



//=================== BATTERY STUFF

long batWindow1[BAT_WINDOW_SIZE];
long batWindow2[BAT_WINDOW_SIZE];

#define B1_R1_RESISTANCE 67.0
#define B1_R2_RESISTANCE 22.2
#define B2_R1_RESISTANCE 67.0
#define B2_R2_RESISTANCE 22.4

long b1Avg;
long b2Avg;


void measureBat1() {
	float raw = (float)analogRead(BATTERY1_PIN);
	float vin = (5.0/1023.0) * raw;
	float vout = (vin * (B1_R1_RESISTANCE + B1_R2_RESISTANCE)) / B1_R2_RESISTANCE;
	int val = vout*1000;
	pushLongArray(batWindow1, BAT_WINDOW_SIZE, val);
	b1Avg = average(batWindow1, BAT_WINDOW_SIZE);
//	vals[ADDR_STATE_BATTERY1] = String(b1Avg, DEC);
	vals[ADDR_STATE_BATTERY1] = String(0, DEC);


}

void measureBat2() {
	float raw = (float)analogRead(BATTERY2_PIN);
	float vin = (5.0/1023.0) * raw;
	float vout = (vin * (B2_R1_RESISTANCE + B2_R2_RESISTANCE)) / B2_R2_RESISTANCE;
	long val = vout*1000;
	pushLongArray(batWindow2, BAT_WINDOW_SIZE, val);
	b2Avg = average(batWindow2, BAT_WINDOW_SIZE);
	//vals[ADDR_STATE_BATTERY2] = String(b2Avg, DEC);
	vals[ADDR_STATE_BATTERY2] = String(0, DEC);

}


bool isRunning() {
	return true;
}


//==========================CHT 
#define PIN_CHT 0
#define CHT_WINDOW_SIZE 8
float HT_R1 = 10.05;
long chtWindow[CHT_WINDOW_SIZE];

void measureCHT() {
	int raw = analogRead(PIN_CHT);    // Reads the Input PIN
	float vout = (5.0 / 1023.0) * raw;    // Calculates the Voltage on th Input PIN
	float buffer = (5.0 / vout) - 1;
	float r2 = HT_R1 / buffer;
	pushLongArray(chtWindow, CHT_WINDOW_SIZE, resToTemp(r2*1000));
	vals[ADDR_CHT] = String(average(chtWindow, CHT_WINDOW_SIZE), DEC);
}


long resToTemp(double res) {
  return (long)(1540 * pow(res, (double)-.38133));
}


//========================== RPM

void measureRPM() {
	vals[ADDR_RPM] = String(0);
}


// #====================== HEAT

void handleHeat(unsigned int address) {
	handler handle = handlers[address];

	//Power can only be 0-255
	unsigned int on   = handle.snippet[3];
	if(on) {
		digitalWrite(HEAT_PIN, LOW);
	} else {
		digitalWrite(HEAT_PIN, HIGH);
	}
}


//========================== UTIL

long average(long* arr, int size) {
	long sum = 0;
	int i = 0;
	for(i; i < size; i++) {
		sum += arr[i];
	}
	return sum/size;
}


void pushLongArray(long* arr, int size, long val) {
	long i = size-1;
	for(i; i > 0; i--) {
		arr[i] = arr[i-1];
	}
	arr[0] = val;
}































