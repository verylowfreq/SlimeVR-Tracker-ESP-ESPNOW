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

#include "GlobalVars.h"

namespace SlimeVR {
namespace Network {

void Manager::setup() {
#ifndef USE_ESPNOW
	::WiFiNetwork::setUp();
#else

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  WiFi.setSleep(false);

  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  } else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
  
  esp_now_peer_info_t slave;
    // マルチキャスト用Slave登録
  memset(&slave, 0, sizeof(slave));
  for (int i = 0; i < 6; ++i) {
    slave.peer_addr[i] = (uint8_t)0xff;
  }
  
  esp_err_t addStatus = esp_now_add_peer(&slave);
  if (addStatus == ESP_OK) {
    // Pair success
    Serial.println("Pair success");
  }
  // ESP-NOWコールバック登録
  // esp_now_register_send_cb(OnDataSent);
//   esp_now_register_recv_cb(OnDataRecv);

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
