/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2017-2020 Mike Dunston

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

class WiFiInterface
{
public:
  WiFiInterface();
  void init();
};

extern WiFiInterface wifiInterface;
void init_webserver(MDNS *mdns);

class WebSocketClient : public DCCPPProtocolConsumer
{
public:
  WebSocketClient(int clientID, uint32_t remoteIP)
    : DCCPPProtocolConsumer(), _id(clientID), _remoteIP(remoteIP)
  {
    LOG(INFO, "[WS %s] Connected", name().c_str());
  }
  virtual ~WebSocketClient()
  {
    LOG(INFO, "[WS %s] Disconnected", name().c_str());
  }
  int id()
  {
    return _id;
  }
  std::string name()
  {
    return StringPrintf("%s/%d", ipv4_to_string(_remoteIP).c_str(), _id);
  }
private:
  uint32_t _id;
  uint32_t _remoteIP;
};

extern vector<unique_ptr<WebSocketClient>> webSocketClients;
extern std::vector<int> jmriClients;

#endif // WIFI_INTERFACE_H_