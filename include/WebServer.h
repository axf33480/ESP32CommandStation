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
#pragma once

#include <os/MDNS.hxx>
#include <ESPAsyncWebServer.h>
#include "InfoScreen.h"

class ESP32CSWebServer : public AsyncWebServer {
public:
  ESP32CSWebServer(MDNS *mdns);
  void begin() {
    _mdns->publish("websvr", "_http._tcp", 80);
    AsyncWebServer::begin();
#if INFO_SCREEN_WS_CLIENTS_LINE >= 0
    InfoScreen::replaceLine(INFO_SCREEN_WS_CLIENTS_LINE, F("WS Clients: 0"));
#endif
  }
  void broadcastToWS(const String &buf) {
    _webSocket.textAll(buf);
  }
private:
  AsyncWebSocket _webSocket;
  MDNS *_mdns;
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
};
