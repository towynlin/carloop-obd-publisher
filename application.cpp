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
const auto OBD_CAN_REPLY_ID_MIN    = 0x7E8;
const auto OBD_CAN_REPLY_ID_MAX    = 0x7EF;

// OBD services / modes
const auto OBD_MODE_CURRENT_DATA = 0x01;

// OBD PIDs
const auto OBD_PID_SUPPORTED_PIDS_01_20                  = 0x00;
// MIL = malfunction indicator lamp = check engine light
const auto OBD_PID_MIL_STATUS                            = 0x01;
const auto OBD_PID_FUEL_SYSTEM_STATUS                    = 0x03;
const auto OBD_PID_ENGINE_LOAD                           = 0x04;
const auto OBD_PID_COOLANT_TEMPERATURE                   = 0x05;
const auto OBD_PID_SHORT_TERM_FUEL_TRIM                  = 0x06;
const auto OBD_PID_LONG_TERM_FUEL_TRIM                   = 0x07;
const auto OBD_PID_ENGINE_RPM                            = 0x0c;
const auto OBD_PID_VEHICLE_SPEED                         = 0x0d;
const auto OBD_PID_TIMING_ADVANCE                        = 0x0e;
const auto OBD_PID_INTAKE_AIR_TEMPERATURE                = 0x0f;
const auto OBD_PID_MAF_AIR_FLOW_RATE                     = 0x10;
const auto OBD_PID_THROTTLE    	                         = 0x11;
const auto OBD_PID_O2_SENSORS_PRESENT                    = 0x13;
const auto OBD_PID_O2_SENSOR_2                           = 0x15;
const auto OBD_PID_OBD_STANDARDS                         = 0x1c;
const auto OBD_PID_ENGINE_RUN_TIME                       = 0x1f;
const auto OBD_PID_SUPPORTED_PIDS_21_40                  = 0x20;
const auto OBD_PID_DISTANCE_TRAVELED_WITH_MIL_ON         = 0x21;
const auto OBD_PID_COMMANDED_EVAPORATIVE_PURGE           = 0x2e;
const auto OBD_PID_FUEL_TANK_LEVEL_INPUT                 = 0x2f;
const auto OBD_PID_WARM_UPS_SINCE_CODES_CLEARED          = 0x30;
const auto OBD_PID_DISTANCE_TRAVELED_SINCE_CODES_CLEARED = 0x31;
const auto OBD_PID_ABS_BAROMETRIC_PRESSURE               = 0x33;
const auto OBD_PID_O2_SENSOR_1                           = 0x34;
const auto OBD_PID_CATALYST_TEMPERATURE_BANK1_SENSOR1    = 0x3c;
const auto OBD_PID_SUPPORTED_PIDS_41_60                  = 0x40;

const size_t NUM_PIDS_TO_REQUEST = 21;
const uint8_t pidsToRequest[NUM_PIDS_TO_REQUEST] = {
	OBD_PID_ENGINE_LOAD,
	OBD_PID_COOLANT_TEMPERATURE,
	OBD_PID_SHORT_TERM_FUEL_TRIM,
	OBD_PID_LONG_TERM_FUEL_TRIM,
	OBD_PID_ENGINE_RPM,
	OBD_PID_VEHICLE_SPEED,
	OBD_PID_TIMING_ADVANCE,
	OBD_PID_INTAKE_AIR_TEMPERATURE,
	OBD_PID_MAF_AIR_FLOW_RATE,
	OBD_PID_THROTTLE,
	OBD_PID_O2_SENSOR_2,
	OBD_PID_ENGINE_RUN_TIME,
	OBD_PID_DISTANCE_TRAVELED_WITH_MIL_ON,
	OBD_PID_COMMANDED_EVAPORATIVE_PURGE,
	OBD_PID_FUEL_TANK_LEVEL_INPUT,
	OBD_PID_WARM_UPS_SINCE_CODES_CLEARED,
	OBD_PID_DISTANCE_TRAVELED_SINCE_CODES_CLEARED,
	OBD_PID_ABS_BAROMETRIC_PRESSURE,
	OBD_PID_O2_SENSOR_1,
	OBD_PID_CATALYST_TEMPERATURE_BANK1_SENSOR1,
	OBD_PID_SUPPORTED_PIDS_41_60
};
uint8_t pidIndex = NUM_PIDS_TO_REQUEST - 1;
const int ledPin = D7;

String dumpForPublish;
int numBufferedMessages = 0;

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
	pidIndex = (pidIndex + 1) % NUM_PIDS_TO_REQUEST;

    CANMessage message;
    message.id = OBD_CAN_BROADCAST_ID;
    message.len = 8; // just always use 8
    message.data[0] = 0x02; // 0 = single-frame format, 2  = num data bytes
    message.data[1] = OBD_MODE_CURRENT_DATA; // OBD MODE
    message.data[2] = pidsToRequest[pidIndex]; // OBD PID

    carloop.can().transmit(message);

    obdLoopFunction = waitForObdResponse;
    transitionTime = millis();
}

void waitForObdResponse() {
    if (millis() - transitionTime >= 100) {
        obdLoopFunction = delayUntilNextRequest;
        transitionTime = millis();
        return;
    }

    bool responseReceived = false;
	String dump;
    CANMessage message;
    while (carloop.can().receive(message)) {
		canMessageCount++;
		if (message.id == 0x130) {
			if (!byteArray8Equal(message.data, lastMessageData)) {
				memcpy(lastMessageData, message.data, 8);
				dump += dumpMessage(message);
				numBufferedMessages++;
			}
		} else {
			if (message.id >= OBD_CAN_REPLY_ID_MIN &&
					message.id <= OBD_CAN_REPLY_ID_MAX &&
					message.data[2] == pidsToRequest[pidIndex]) {
				responseReceived = true;
			}
			dump += dumpMessage(message);
			numBufferedMessages++;
		}
	}

	Serial.write(dump);
	dumpForPublish += dump;
	if (numBufferedMessages >= 9) {
		Particle.publish("m", dumpForPublish, 60, PRIVATE);
		numBufferedMessages = 0;
		dumpForPublish.remove(0);
	}

    if (responseReceived) {
        digitalWrite(ledPin, HIGH);
    }
}

void delayUntilNextRequest() {
    if (millis() - transitionTime >= 80) {
        obdLoopFunction = sendObdRequest;
        transitionTime = millis();
    } else if (millis() - transitionTime >= 20) {
        digitalWrite(ledPin, LOW);
    }
}

/*************** End: OBD Loop Functions ****************/


void printValuesAtInterval() {
	static const unsigned long interval = 20000;
	static unsigned long lastDisplay = 0;
	if (millis() - lastDisplay < interval) {
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
	String str = String::format("%.2f:", millis() / 1000.0);
	for (int i = 0; i < message.len; i++) {
		str += String::format("%02x", message.data[i]);
	}
	str += ",";
	return str;
}

bool byteArray8Equal(uint8_t a1[8], uint8_t a2[8]) {
	for (int i = 0; i < 8; i++) {
		if (a1[i] != a2[i]) return false;
	}
	return true;
}
