#include "application.h"
#include "carloop.h"

SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

void sendObdRequest();
void waitForObdResponse();
void delayUntilNextRequest();
void printValuesAtInterval();
void printValues();
String dumpMessage(const CANMessage &message);
bool byteArray8Equal(uint8_t a1[8], uint8_t a2[8]);

Carloop<CarloopRevision2> carloop;

int canMessageCount = 0;

// OBD CAN message IDs
const auto OBD_CAN_BROADCAST_ID    = 0X7DF;
const auto OBD_CAN_REQUEST_ID      = 0x7E0;
const auto OBD_CAN_REPLY_ID        = 0x7E8;

// OBD services / modes
const auto OBD_MODE_CURRENT_DATA = 0x01;

// OBD PIDs
const auto OBD_PID_SUPPORTED_PIDS      = 0x00;
// MIL = malfunction indicator lamp = check engine light
const auto OBD_PID_MIL_STATUS          = 0x01;
const auto OBD_PID_ENGINE_COOLANT_TEMP = 0x05;
const auto OBD_PID_ENGINE_RPM          = 0x0C;
const auto OBD_PID_VEHICLE_SPEED       = 0x0D;
const auto OBD_PID_MAF_SENSOR          = 0x10;
const auto OBD_PID_O2_VOLTAGE          = 0x14;
const auto OBD_PID_THROTTLE    	       = 0x11;


const uint8_t pid = OBD_PID_MIL_STATUS;
const int ledPin = D7;

auto *obdLoopFunction = sendObdRequest;
unsigned long transitionTime = 0;
uint8_t lastMessageData[8];

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


/*************** Begin: OBD Loop Functions ****************/

/* For help understanding the OBD Query format over CAN bus,
 * see: https://en.wikipedia.org/wiki/OBD-II_PIDs#Query
 *
 * For help understanding why the first data byte is 0x02,
 * see: http://hackaday.com/2013/10/29/can-hacking-protocols/
 *
 * For help understanding modes and PIDs,
 * see: https://en.wikipedia.org/wiki/OBD-II_PIDs#Modes
 * and: https://en.wikipedia.org/wiki/OBD-II_PIDs#Standard_PIDs
 */
void sendObdRequest() {
    CANMessage message;
    message.id = OBD_CAN_BROADCAST_ID;
    message.len = 8; // just always use 8
    message.data[0] = 0x02; // 0 = single-frame format, 2  = num data bytes
    message.data[1] = OBD_MODE_CURRENT_DATA; // OBD MODE
    message.data[2] = pid; // OBD PID
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
		canMessageCount++;
		if(message.id == OBD_CAN_REPLY_ID && message.data[2] == pid) {
			responseReceived = true;
		}
		if (message.id == 0x130) {
			if (!byteArray8Equal(message.data, lastMessageData)) {
				memcpy(lastMessageData, message.data, 8);
				dump += dumpMessage(message);
			}
		} else {
			dump += dumpMessage(message);
		}
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

/*************** End: OBD Loop Functions ****************/


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
	Serial.printf("Battery voltage: %12f ", carloop.battery());
	Serial.printf("CAN messages: %12d ", canMessageCount);
	Serial.println("");
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

bool byteArray8Equal(uint8_t a1[8], uint8_t a2[8]) {
	for (int i = 0; i < 8; i++) {
		if (a1[i] != a2[i]) return false;
	}
	return true;
}
