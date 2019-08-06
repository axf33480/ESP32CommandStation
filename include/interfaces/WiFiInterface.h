/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2017-2019 Mike Dunston

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see http://www.gnu.org/licenses
**********************************************************************/

#ifndef WIFI_INTERFACE_H_
#define WIFI_INTERFACE_H_

class WiFiInterface {
public:
  WiFiInterface();
  void init();
  void showConfiguration();
  void showInitInfo();
  void broadcast(const std::string &);
  void setIP(tcpip_adapter_ip_info_t ip) {
    ip_.ip = ip.ip;
  }
private:
  tcpip_adapter_ip_info_t ip_;
};

extern WiFiInterface wifiInterface;

class ESP32CSWebServer {
public:
  ESP32CSWebServer(MDNS *mdns);
  void begin();
  void broadcastToWS(const std::string &buf);
private:
  MDNS *mdns_;
  std::string softAPAddress_;
  std::string stationAddress_;
  std::vector<uint32_t> captiveIPs_;
  void handleESPInfo(AsyncWebServerRequest *);
  void handleProgrammer(AsyncWebServerRequest *);
  void handlePower(AsyncWebServerRequest *);
  void handleOutputs(AsyncWebServerRequest *);
  void handleTurnouts(AsyncWebServerRequest *);
  void handleSensors(AsyncWebServerRequest *);
  void handleConfig(AsyncWebServerRequest *);
  void handleLocomotive(AsyncWebServerRequest *);
#if S88_ENABLED
  void handleS88Sensors(AsyncWebServerRequest *);
#endif
  void handleRemoteSensors(AsyncWebServerRequest *);
  void handleOTA(AsyncWebServerRequest *);
  void handleFeatures(AsyncWebServerRequest *);
  void streamResource(AsyncWebServerRequest *);
  void notFoundHandler(AsyncWebServerRequest *);
};

class WebSocketClient : public DCCPPProtocolConsumer {
public:
  WebSocketClient(int clientID, uint32_t remoteIP) : _id(clientID), _remoteIP(remoteIP) {
    LOG(INFO, "[WS %s] Connected", getName().c_str());
  }
  virtual ~WebSocketClient() {
    LOG(INFO, "[WS %s] Disconnected", getName().c_str());
  }
  int getID() {
    return _id;
  }
  std::string getName() {
    return StringPrintf("%s/%d", ipv4_to_string(_remoteIP).c_str(), _id);
  }
private:
  uint32_t _id;
  uint32_t _remoteIP;
};

extern std::vector<WebSocketClient *> webSocketClients;
extern std::vector<int> jmriClients;

#endif // WIFI_INTERFACE_H_