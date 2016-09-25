/*
 * Copyright 2016 Zachary Crockett
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "application.h"
#include "carloop.h"
#include "base85.h"

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
const auto OBD_PID_ABSOLUTE_BAROMETRIC_PRESSURE          = 0x33;
const auto OBD_PID_O2_SENSOR_1                           = 0x34;
const auto OBD_PID_CATALYST_TEMPERATURE_BANK1_SENSOR1    = 0x3c;
const auto OBD_PID_SUPPORTED_PIDS_41_60                  = 0x40;
const auto OBD_PID_MONITOR_STATUS                        = 0X41;
const auto OBD_PID_CONTROL_MODULE_VOLTAGE                = 0X42;
const auto OBD_PID_ABSOLUTE_LOAD_VALUE                   = 0X43;
const auto OBD_PID_FUEL_AIR_COMMANDED_EQUIV_RATIO        = 0X44;
const auto OBD_PID_RELATIVE_THROTTLE                     = 0X45;
const auto OBD_PID_AMBIENT_AIR_TEMPERATURE               = 0X46;
const auto OBD_PID_ABSOLUTE_THROTTLE_B                   = 0X47;
const auto OBD_PID_ACCELERATOR_PEDAL_POSITION_D          = 0X49;
const auto OBD_PID_ACCELERATOR_PEDAL_POSITION_E          = 0X4a;
const auto OBD_PID_COMMANDED_THROTTLE_ACTUATOR           = 0X4c;

const size_t NUM_PIDS_TO_REQUEST = 30;
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
	OBD_PID_ABSOLUTE_BAROMETRIC_PRESSURE,
	OBD_PID_O2_SENSOR_1,
	OBD_PID_CATALYST_TEMPERATURE_BANK1_SENSOR1,
	OBD_PID_MONITOR_STATUS,
	OBD_PID_CONTROL_MODULE_VOLTAGE,
	OBD_PID_ABSOLUTE_LOAD_VALUE,
	OBD_PID_FUEL_AIR_COMMANDED_EQUIV_RATIO,
	OBD_PID_RELATIVE_THROTTLE,
	OBD_PID_AMBIENT_AIR_TEMPERATURE,
	OBD_PID_ABSOLUTE_THROTTLE_B,
	OBD_PID_ACCELERATOR_PEDAL_POSITION_D,
	OBD_PID_ACCELERATOR_PEDAL_POSITION_E,
	OBD_PID_COMMANDED_THROTTLE_ACTUATOR
};
uint8_t pidIndex = NUM_PIDS_TO_REQUEST - 1;

String dumpForPublish;

auto *obdLoopFunction = sendObdRequest;
unsigned long transitionTime = 0;
uint8_t lastMessageData[8];

void setup() {
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

	String dump;
	CANMessage message;
	while (carloop.can().receive(message)) {
		canMessageCount++;
		if (message.id == 0x130) {
			// This message gets sent every tenth of a second,
			// so only publish it when the value changes.
			if (!byteArray8Equal(message.data, lastMessageData)) {
				memcpy(lastMessageData, message.data, 8);
				dump += dumpMessage(message);
			}
		} else {
			dump += dumpMessage(message);
		}
	}

	Serial.write(dump);
	dumpForPublish += dump;
	// change to:
	// if > 196 binary bytes
	// when each time stamp is exactly 2 bytes encoding integer tenths of a second
	// and 1-5 data bytes follow
	// then we base85 encode the binary to get close to the 255-byte publish limit
	if (dumpForPublish.length() >= 220) {
		Particle.publish("m", dumpForPublish, 60, PRIVATE);
		dumpForPublish.remove(0);
	}
}

void delayUntilNextRequest() {
	if (millis() - transitionTime >= 80) {
		obdLoopFunction = sendObdRequest;
		transitionTime = millis();
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
	// change to: each timestamp as exactly 2 bytes
	// as unsigned integer tenths of a second
	// wrap around carefully
	// maybe (quick stab) -- uint16_t t = (millis() / 100) & 0xffff
	String str = String::format("%.1f:", millis() / 1000.0);
	int startIdx = 0;
	int lastIdx = message.len - 1;
	if (message.id >= 0x700) {
		// We can ignore the first two bytes
		// data[0] is the length
		// data[1] is the UDS service response ID, always 0x41 for me
		// data[2] is important to include - it's the PID
		startIdx = 2;
		if (message.data[0] < message.len) {
			lastIdx = message.data[0];
		}
	}
	// hmm... Since the binary data can include zeros
	// I may have to do the base85 encoding on the fly
	// rather than right before publish...
	for (int i = startIdx; i <= lastIdx; i++) {
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
