#include "application.h"
#include "carloop.h"

SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

void receiveCanMessages();
void sendObdRequest();
void waitForObdResponse();
void delayUntilNextRequest();
void printValuesAtInterval();
void printValues();
void printFloat(float val, bool valid, int len, int prec);
void printDateTime(TinyGPSDate &d, TinyGPSTime &t);
String dumpMessage(const CANMessage &message);

Carloop<CarloopRevision2> carloop;

int canMessageCount = 0;

// ODB CAN message IDs
const auto OBD_REQUEST_ID      = 0x7E0;
const auto OBD_REPLY_ID        = 0x7E8;

const auto OBD_PID_SERVICE     = 0x01;

// ODB PID constants
const auto ENGINE_COOLANT_TEMP = 0x05;
const auto ENGINE_RPM          = 0x0C;
const auto VEHICLE_SPEED       = 0x0D;
const auto MAF_SENSOR          = 0x10;
const auto O2_VOLTAGE          = 0x14;
const auto THROTTLE    	       = 0x11;

const uint8_t pid = ENGINE_RPM;
const int ledPin = D7;

auto *obdLoopFunction = sendObdRequest;
unsigned long transitionTime = 0;

void setup() {
	pinMode(ledPin, OUTPUT);
	Serial.begin(115200);
	carloop.begin();
	Particle.connect();
	transitionTime = millis();
}

void loop() {
	carloop.update();
	printValuesAtInterval();
	obdLoopFunction();
}

// currently unused
void receiveCanMessages() {
	String dump;
	CANMessage message;
	while(carloop.can().receive(message)) {
		canMessageCount++;
		// If the serial port or TCP server is overwhelmed, limit messages
		// if(message.id >= 0x700)
		dump += dumpMessage(message);
	}
	Serial.write(dump);
}


/* Begin: OBD Loop Functions */

void sendObdRequest() {
    CANMessage message;
    message.id = OBD_REQUEST_ID;
    message.len = 8;
    message.data[0] = 0x02;
    message.data[1] = OBD_PID_SERVICE;
    message.data[2] = pid;
    carloop.can().transmit(message);

    obdLoopFunction = waitForObdResponse;
    transitionTime = millis();
}

void waitForObdResponse() {
    if(millis() - transitionTime >= 100) {
        obdLoopFunction = delayUntilNextRequest;
        transitionTime = millis();
        return;
    }

    bool responseReceived = false;
	String dump;
    CANMessage message;
    while(carloop.can().receive(message)) {
        if(message.id == OBD_REPLY_ID && message.data[2] == pid) {
            responseReceived = true;
        }
		canMessageCount++;
		// If the serial port is overwhelmed, limit messages
		// if(message.id >= 0x700)
		dump += dumpMessage(message);
    }
	Serial.write(dump);

    if(responseReceived) {
        digitalWrite(ledPin, HIGH);
    }
}

void delayUntilNextRequest() {
    if(millis() - transitionTime >= 400) {
        obdLoopFunction = sendObdRequest;
        transitionTime = millis();
    } else if(millis() - transitionTime >= 250) {
        digitalWrite(ledPin, LOW);
    }
}

/* End: OBD Loop Functions */


void printValuesAtInterval() {
	static const unsigned long interval = 20000;
	static unsigned long lastDisplay = 0;
	if(millis() - lastDisplay < interval) {
		return;
	}
	lastDisplay = millis();
	printValues();
}

void printValues() {
	//auto &gps = carloop.gps();
	Serial.printf("Battery voltage: %12f ", carloop.battery());
	Serial.printf("CAN messages: %12d ", canMessageCount);
	//Serial.printf("GPS %6d chars: ", gps.charsProcessed());
	//printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
	//printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
	//printDateTime(gps.date, gps.time);
	Serial.println("");
}

void printFloat(float val, bool valid, int len, int prec) {
	if(valid) {
		char format[10];
		snprintf(format, sizeof(format), "%%%d.%df", len, prec);
		Serial.printf(format, val);
	} else {
		while(len-- > 1) {
			Serial.print('*');
		}
		Serial.print(' ');
	}
}

void printDateTime(TinyGPSDate &d, TinyGPSTime &t) {
	if (d.isValid()) {
		Serial.printf("%02d/%02d/%02d ", d.month(), d.day(), d.year());
	} else {
		Serial.print("********** ");
	}

	if (t.isValid()) {
		Serial.printf("%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
	} else {
		Serial.print("******** ");
	}
}

String dumpMessage(const CANMessage &message) {
	String str = String::format("{\"timestamp\":%f,\"id\":\"0x%03x\",\"data\":\"", millis() / 1000.0, message.id, message.len);
	for(int i = 0; i < message.len; i++) {
		if(i == 0) {
			str += "0x";
		}
		str += String::format("%02x", message.data[i]);
	}
	str += "\"}\n";
	return str;
}
