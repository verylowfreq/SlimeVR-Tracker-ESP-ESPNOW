/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2023 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/

#include "connection_espnow.h"

#include <esp_now.h>

#include "GlobalVars.h"
#include "logging/Logger.h"
#include "packets.h"

#define TIMEOUT 3000UL

template <typename T>
unsigned char* convert_to_chars(T src, unsigned char* target) {
	union uwunion {
		unsigned char c[sizeof(T)];
		T v;
	} un;
	un.v = src;
	for (size_t i = 0; i < sizeof(T); i++) {
		target[i] = un.c[sizeof(T) - i - 1];
	}
	return target;
}

template <typename T>
T convert_chars(unsigned char* const src) {
	union uwunion {
		unsigned char c[sizeof(T)];
		T v;
	} un;
	for (size_t i = 0; i < sizeof(T); i++) {
		un.c[i] = src[sizeof(T) - i - 1];
	}
	return un.v;
}

namespace SlimeVR {
namespace Network {

#define MUST_TRANSFER_BOOL(b) \
	if (!b)                   \
		return false;

#define MUST(b) \
	if (!b)     \
		return;

bool ConnectionESPNOW::beginPacket() {
	// if (m_IsBundle) {
	// 	m_BundlePacketPosition = 0;
	// 	return true;
	// }

	// int r = m_UDP.beginPacket(m_ServerHost, m_ServerPort);
	// if (r == 0) {
	// 	// This *technically* should *never* fail, since the underlying UDP
	// 	// library just returns 1.

	// 	m_Logger.warn("UDP beginPacket() failed");
	// }

	// return r > 0;

    memset(sendBuffer, 0x00, sizeof(sendBuffer));
    memcpy(sendBuffer, "SLVR\0", 5);
    dataLength = 5;

	return true;
}

bool ConnectionESPNOW::endPacket() {
	// if (m_IsBundle) {
	// 	uint32_t innerPacketSize = m_BundlePacketPosition;

	// 	MUST_TRANSFER_BOOL((innerPacketSize > 0));

	// 	m_IsBundle = false;
		
	// 	if (m_BundlePacketInnerCount == 0) {
	// 		sendPacketType(PACKET_BUNDLE);
	// 		sendPacketNumber();
	// 	}
	// 	sendShort(innerPacketSize);
	// 	sendBytes(m_Packet, innerPacketSize);

	// 	m_BundlePacketInnerCount++;
	// 	m_IsBundle = true;
	// 	return true;
	// }

	// int r = m_UDP.endPacket();
	// if (r == 0) {
	// 	// This is usually just `ERR_ABRT` but the UDP client doesn't expose
	// 	// the full error code to us, so we just have to live with it.

	// 	// m_Logger.warn("UDP endPacket() failed");
	// }

	// return r > 0;

    esp_now_send(peer_addr, sendBuffer, dataLength);

    return true;
}

bool ConnectionESPNOW::beginBundle() {
	// MUST_TRANSFER_BOOL(m_ServerFeatures.has(ServerFeatures::PROTOCOL_BUNDLE_SUPPORT));
	// MUST_TRANSFER_BOOL(m_Connected);
	// MUST_TRANSFER_BOOL(!m_IsBundle);
	// MUST_TRANSFER_BOOL(beginPacket());

	// m_IsBundle = true;
	// m_BundlePacketInnerCount = 0;
	return true;
}

bool ConnectionESPNOW::endBundle() {
	// MUST_TRANSFER_BOOL(m_IsBundle);

	// m_IsBundle = false;
	
	// MUST_TRANSFER_BOOL((m_BundlePacketInnerCount > 0));

	return endPacket();
}

size_t ConnectionESPNOW::write(const uint8_t *buffer, size_t size) {
	// if (m_IsBundle) {
	// 	if (m_BundlePacketPosition + size > sizeof(m_Packet)) {
	// 		return 0;
	// 	}
	// 	memcpy(m_Packet + m_BundlePacketPosition, buffer, size);
	// 	m_BundlePacketPosition += size;
	// 	return size;
	// }
	// return m_UDP.write(buffer, size);

    uint8_t* dst = &sendBuffer[dataLength];
    memcpy(dst, buffer, size);
    dataLength += size;

    return size;
}

size_t ConnectionESPNOW::write(uint8_t byte) {
	return write(&byte, 1);
}

bool ConnectionESPNOW::sendFloat(float f) {
	convert_to_chars(f, m_Buf);

	return write(m_Buf, sizeof(f)) != 0;
}

bool ConnectionESPNOW::sendByte(uint8_t c) { return write(&c, 1) != 0; }

bool ConnectionESPNOW::sendShort(uint16_t i) {
	convert_to_chars(i, m_Buf);

	return write(m_Buf, sizeof(i)) != 0;
}

bool ConnectionESPNOW::sendInt(uint32_t i) {
	convert_to_chars(i, m_Buf);

	return write(m_Buf, sizeof(i)) != 0;
}

bool ConnectionESPNOW::sendLong(uint64_t l) {
	convert_to_chars(l, m_Buf);

	return write(m_Buf, sizeof(l)) != 0;
}

bool ConnectionESPNOW::sendBytes(const uint8_t* c, size_t length) {
	return write(c, length) != 0;
}

bool ConnectionESPNOW::sendPacketNumber() {
	if (m_IsBundle) {
		return true;
	}

	uint64_t pn = m_PacketNumber++;

	return sendLong(pn);
}

bool ConnectionESPNOW::sendShortString(const char* str) {
	uint8_t size = strlen(str);

	MUST_TRANSFER_BOOL(sendByte(size));
	MUST_TRANSFER_BOOL(sendBytes((const uint8_t*)str, size));

	return true;
}

bool ConnectionESPNOW::sendPacketType(uint8_t type) {
	MUST_TRANSFER_BOOL(sendByte(0));
	MUST_TRANSFER_BOOL(sendByte(0));
	MUST_TRANSFER_BOOL(sendByte(0));

	return sendByte(type);
}

bool ConnectionESPNOW::sendLongString(const char* str) {
	int size = strlen(str);

	MUST_TRANSFER_BOOL(sendInt(size));

	return sendBytes((const uint8_t*)str, size);
}

int ConnectionESPNOW::getWriteError() {
#ifndef USE_ESPNOW
	return m_UDP.getWriteError();
#endif
	return 0;
}

// PACKET_HEARTBEAT 0
void ConnectionESPNOW::sendHeartbeat() {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_HEARTBEAT));
	MUST(sendPacketNumber());

	MUST(endPacket());
}

// PACKET_ACCEL 4
void ConnectionESPNOW::sendSensorAcceleration(uint8_t sensorId, Vector3 vector) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_ACCEL));
	MUST(sendPacketNumber());
	MUST(sendFloat(vector.x));
	MUST(sendFloat(vector.y));
	MUST(sendFloat(vector.z));
	MUST(sendByte(sensorId));

	MUST(endPacket());
}

// PACKET_BATTERY_LEVEL 12
void ConnectionESPNOW::sendBatteryLevel(float batteryVoltage, float batteryPercentage) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_BATTERY_LEVEL));
	MUST(sendPacketNumber());
	MUST(sendFloat(batteryVoltage));
	MUST(sendFloat(batteryPercentage));

	MUST(endPacket());
}

// PACKET_TAP 13
void ConnectionESPNOW::sendSensorTap(uint8_t sensorId, uint8_t value) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_TAP));
	MUST(sendPacketNumber());
	MUST(sendByte(sensorId));
	MUST(sendByte(value));

	MUST(endPacket());
}

// PACKET_ERROR 14
void ConnectionESPNOW::sendSensorError(uint8_t sensorId, uint8_t error) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_ERROR));
	MUST(sendPacketNumber());
	MUST(sendByte(sensorId));
	MUST(sendByte(error));

	MUST(endPacket());
}

// PACKET_SENSOR_INFO 15
void ConnectionESPNOW::sendSensorInfo(Sensor* sensor) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_SENSOR_INFO));
	MUST(sendPacketNumber());
	MUST(sendByte(sensor->getSensorId()));
	MUST(sendByte((uint8_t)sensor->getSensorState()));
	MUST(sendByte(sensor->getSensorType()));

	MUST(endPacket());
}

// PACKET_ROTATION_DATA 17
void ConnectionESPNOW::sendRotationData(
	uint8_t sensorId,
	Quat* const quaternion,
	uint8_t dataType,
	uint8_t accuracyInfo
) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_ROTATION_DATA));
	MUST(sendPacketNumber());
	MUST(sendByte(sensorId));
	MUST(sendByte(dataType));
	MUST(sendFloat(quaternion->x));
	MUST(sendFloat(quaternion->y));
	MUST(sendFloat(quaternion->z));
	MUST(sendFloat(quaternion->w));
	MUST(sendByte(accuracyInfo));

	MUST(endPacket());
}

// PACKET_MAGNETOMETER_ACCURACY 18
void ConnectionESPNOW::sendMagnetometerAccuracy(uint8_t sensorId, float accuracyInfo) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_MAGNETOMETER_ACCURACY));
	MUST(sendPacketNumber());
	MUST(sendByte(sensorId));
	MUST(sendFloat(accuracyInfo));

	MUST(endPacket());
}

// PACKET_SIGNAL_STRENGTH 19
void ConnectionESPNOW::sendSignalStrength(uint8_t signalStrength) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_SIGNAL_STRENGTH));
	MUST(sendPacketNumber());
	MUST(sendByte(255));
	MUST(sendByte(signalStrength));

	MUST(endPacket());
}

// PACKET_TEMPERATURE 20
void ConnectionESPNOW::sendTemperature(uint8_t sensorId, float temperature) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_TEMPERATURE));
	MUST(sendPacketNumber());
	MUST(sendByte(sensorId));
	MUST(sendFloat(temperature));

	MUST(endPacket());
}

// PACKET_FEATURE_FLAGS 22
void ConnectionESPNOW::sendFeatureFlags() {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_FEATURE_FLAGS));
	MUST(sendPacketNumber());
	MUST(write(FirmwareFeatures::flags.data(), FirmwareFeatures::flags.size()));

	MUST(endPacket());
}

void ConnectionESPNOW::sendTrackerDiscovery() {
	MUST(!m_Connected);

	uint8_t mac[6];
	WiFi.macAddress(mac);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_HANDSHAKE));
	// Packet number is always 0 for handshake
	MUST(sendLong(0));
	MUST(sendInt(BOARD));
	// This is kept for backwards compatibility,
	// but the latest SlimeVR server will not initialize trackers
	// with firmware build > 8 until it recieves a sensor info packet
	MUST(sendInt(IMU));
	MUST(sendInt(HARDWARE_MCU));
	MUST(sendInt(0));
	MUST(sendInt(0));
	MUST(sendInt(0));
	MUST(sendInt(FIRMWARE_BUILD_NUMBER));
	MUST(sendShortString(FIRMWARE_VERSION));
	// MAC address string
	MUST(sendBytes(mac, 6));

	MUST(endPacket());
}

#if ENABLE_INSPECTION
void ConnectionESPNOW::sendInspectionRawIMUData(
	uint8_t sensorId,
	int16_t rX,
	int16_t rY,
	int16_t rZ,
	uint8_t rA,
	int16_t aX,
	int16_t aY,
	int16_t aZ,
	uint8_t aA,
	int16_t mX,
	int16_t mY,
	int16_t mZ,
	uint8_t mA
) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_INSPECTION));
	MUST(sendPacketNumber());

	MUST(sendByte(PACKET_INSPECTION_PACKETTYPE_RAW_IMU_DATA));

	MUST(sendByte(sensorId));
	MUST(sendByte(PACKET_INSPECTION_DATATYPE_INT));

	MUST(sendInt(rX));
	MUST(sendInt(rY));
	MUST(sendInt(rZ));
	MUST(sendByte(rA));

	MUST(sendInt(aX));
	MUST(sendInt(aY));
	MUST(sendInt(aZ));
	MUST(sendByte(aA));

	MUST(sendInt(mX));
	MUST(sendInt(mY));
	MUST(sendInt(mZ));
	MUST(sendByte(mA));

	MUST(endPacket());
}

void ConnectionESPNOW::sendInspectionRawIMUData(
	uint8_t sensorId,
	float rX,
	float rY,
	float rZ,
	uint8_t rA,
	float aX,
	float aY,
	float aZ,
	uint8_t aA,
	float mX,
	float mY,
	float mZ,
	uint8_t mA
) {
	MUST(m_Connected);

	MUST(beginPacket());

	MUST(sendPacketType(PACKET_INSPECTION));
	MUST(sendPacketNumber());

	MUST(sendByte(PACKET_INSPECTION_PACKETTYPE_RAW_IMU_DATA));

	MUST(sendByte(sensorId));
	MUST(sendByte(PACKET_INSPECTION_DATATYPE_FLOAT));

	MUST(sendFloat(rX));
	MUST(sendFloat(rY));
	MUST(sendFloat(rZ));
	MUST(sendByte(rA));

	MUST(sendFloat(aX));
	MUST(sendFloat(aY));
	MUST(sendFloat(aZ));
	MUST(sendByte(aA));

	MUST(sendFloat(mX));
	MUST(sendFloat(mY));
	MUST(sendFloat(mZ));
	MUST(sendByte(mA));

	MUST(endPacket());
}
#endif

void ConnectionESPNOW::returnLastPacket(int len) {

}

void ConnectionESPNOW::updateSensorState(std::vector<Sensor *> & sensors) {
	if (millis() - m_LastSensorInfoPacketTimestamp <= 1000) {
		return;
	}

	m_LastSensorInfoPacketTimestamp = millis();

	for (int i = 0; i < (int)sensors.size(); i++) {
		if (m_AckedSensorState[i] != sensors[i]->getSensorState()) {
			sendSensorInfo(sensors[i]);
		}
	}
}

void ConnectionESPNOW::maybeRequestFeatureFlags() {	
	if (m_ServerFeatures.isAvailable() || m_FeatureFlagsRequestAttempts >= 15) {
		return;
	}

	if (millis() - m_FeatureFlagsRequestTimestamp < 500) {
		return;
	}

	sendFeatureFlags();
	m_FeatureFlagsRequestTimestamp = millis();
	m_FeatureFlagsRequestAttempts++;
}

void ConnectionESPNOW::searchForServer() {
    // ESPNOW doesnt search the server. Send the data anytime.
    // So this is dummy function.

    memset(peer_addr, 0xff, 6);

    m_LastPacketTimestamp = millis();
    m_Connected = true;
    
    m_FeatureFlagsRequestAttempts = 0;
    m_ServerFeatures = ServerFeatures { };

    statusManager.setStatus(SlimeVR::Status::SERVER_CONNECTING, false);
    ledManager.off();
}

void ConnectionESPNOW::reset() {
	m_Connected = false;
	std::fill(m_AckedSensorState, m_AckedSensorState+MAX_IMU_COUNT, SensorStatus::SENSOR_OFFLINE);

	statusManager.setStatus(SlimeVR::Status::SERVER_CONNECTING, true);
}

void ConnectionESPNOW::update() {
	std::vector<Sensor *> & sensors = sensorManager.getSensors();

	updateSensorState(sensors);
	maybeRequestFeatureFlags();

	if (!m_Connected) {
		searchForServer();
		return;
	}

#ifndef USE_ESPNOW
	if (m_LastPacketTimestamp + TIMEOUT < millis()) {
		statusManager.setStatus(SlimeVR::Status::SERVER_CONNECTING, true);

		m_Connected = false;
		std::fill(m_AckedSensorState, m_AckedSensorState+MAX_IMU_COUNT, SensorStatus::SENSOR_OFFLINE);
		m_Logger.warn("Connection to server timed out");

		return;
	}

	int packetSize = m_UDP.parsePacket();
	if (!packetSize) {
		return;
	}

	m_LastPacketTimestamp = millis();
	int len = m_UDP.read(m_Packet, sizeof(m_Packet));

#ifdef DEBUG_NETWORK
	m_Logger.trace(
		"Received %d bytes from %s, port %d",
		packetSize,
		m_UDP.remoteIP().toString().c_str(),
		m_UDP.remotePort()
	);
	m_Logger.traceArray("UDP packet contents: ", m_Packet, len);
#else
	(void)packetSize;
#endif

	switch (convert_chars<int>(m_Packet)) {
		case PACKET_RECEIVE_HEARTBEAT:
			sendHeartbeat();
			break;

		case PACKET_RECEIVE_VIBRATE:
			break;

		case PACKET_RECEIVE_HANDSHAKE:
			// Assume handshake successful
			m_Logger.warn("Handshake received again, ignoring");
			break;

		case PACKET_RECEIVE_COMMAND:
			break;

		case PACKET_CONFIG:
			break;

		case PACKET_PING_PONG:
			returnLastPacket(len);
			break;

		case PACKET_SENSOR_INFO:
			if (len < 6) {
				m_Logger.warn("Wrong sensor info packet");
				break;
			}

			for (int i = 0; i < (int)sensors.size(); i++) {
				if (m_Packet[4] == sensors[i]->getSensorId()) {
					m_AckedSensorState[i] = (SensorStatus)m_Packet[5];
					break;
				}
			}

			break;

		case PACKET_FEATURE_FLAGS:
			// Packet type (4) + Packet number (8) + flags (len - 12)
			if (len < 13) {
				m_Logger.warn("Invalid feature flags packet: too short");
				break;
			}

			bool hadFlags = m_ServerFeatures.isAvailable();
			
			uint32_t flagsLength = len - 12;
			m_ServerFeatures = ServerFeatures::from(&m_Packet[12], flagsLength);

			if (!hadFlags) {
				#if PACKET_BUNDLING != PACKET_BUNDLING_DISABLED
					if (m_ServerFeatures.has(ServerFeatures::PROTOCOL_BUNDLE_SUPPORT)) {
						m_Logger.debug("Server supports packet bundling");
					}
				#endif
			}

			break;
#endif
}

}  // namespace Network
}  // namespace SlimeVR
