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

class WiFiInterface {
public:
  WiFiInterface();
  void init();
  void showConfiguration();
  void showInitInfo();
  void send(const String &);
  void print(const __FlashStringHelper *fmt, ...);
  void setIP(tcpip_adapter_ip_info_t ip) {
    _ip_info.ip = ip.ip;
  }
private:
  tcpip_adapter_ip_info_t _ip_info;
};

extern WiFiInterface wifiInterface;
