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
#include "manager.h"
#include "esp_now.h"
#include <Preferences.h>

#include "GlobalVars.h"

namespace SlimeVR {
namespace Network {

void Manager::setup() {
#ifndef USE_ESPNOW
	::WiFiNetwork::setUp();
#else

	WiFi.mode(WIFI_STA);
	WiFi.disconnect();

	// FIXME: Is it really needed?
	WiFi.setSleep(false);

	if (esp_now_init() == ESP_OK) {
		Serial.println("ESPNow Init Success");
	} else {
		Serial.println("ESPNow Init Failed");
		ESP.restart();
	}

	esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_54M);

	esp_now_peer_info_t peer = { 0 };
	peer.channel = 1;
	uint8_t macaddr[6] = { 0 };
	if (preferences.getBytes("PEER_MACADDR", macaddr, 6) != 6) {
		Serial.println("[ESPNOW] Peer MAC address not configured.");
	} else {
		memcpy(&peer.peer_addr, macaddr, 6);
		Serial.printf("[ESPNOW] Peer address: %02x:%02x:%02x:%02x:%02x:%02x\n", peer.peer_addr[0],peer.peer_addr[1],peer.peer_addr[2],peer.peer_addr[3],peer.peer_addr[4],peer.peer_addr[5]);
	}

	if (esp_now_add_peer(&peer) != ESP_OK) {
		Serial.println("Add peer for ESP-NOW failed.");
		// ESP.restart();
	}

	// Transmission over ESP-NOW is always enabled.
	m_IsConnected = true;
#endif
}

void Manager::update() {
#ifndef USE_ESPNOW
	WiFiNetwork::upkeep();
#endif

	auto wasConnected = m_IsConnected;

#ifndef USE_ESPNOW
	m_IsConnected = ::WiFiNetwork::isConnected();
#endif

	if (!m_IsConnected) {
		return;
	}

	if (!wasConnected) {
		// WiFi was reconnected, rediscover the server and reconnect
		networkConnection.reset();
	}

	networkConnection.update();
}

}  // namespace Network
}  // namespace SlimeVR
