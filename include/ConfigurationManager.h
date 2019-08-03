/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2018-2019 Mike Dunston

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

#ifndef CONFIG_MGR_H_
#define CONFIG_MGR_H_

#include <ArduinoJson.h>
#include <openlcb/Defs.hxx>

// Class definition for the Configuration Management system in ESP32 Command Station
class ConfigurationManager
{
public:
  ConfigurationManager();
  virtual ~ConfigurationManager();
  void init();
  void clear();

  bool exists(const char *);
  void remove(const char *);
  JsonObject load(const char *);
  JsonObject load(const char *, DynamicJsonDocument &);
  void store(const char *, const JsonObject);
  JsonObject createRootNode();
  openlcb::NodeID getNodeId();
  void configureCAN(OpenMRN *openmrn);
  void configureWiFi(openlcb::SimpleCanStack *, const WiFiConfiguration &);
  std::string getCSConfig();
  std::string getSSID()
  {
    return wifiSSID_;
  }
  bool isAPEnabled()
  {
    return wifiMode_ == WIFI_MODE_AP || wifiMode_ == WIFI_MODE_APSTA;
  }
private:
  DynamicJsonDocument jsonBuffer{20480};
  std::string getFilePath(const char *, bool=false);
  std::string csConfig_{""};
  std::string wifiSSID_{SSID_NAME};
  std::string wifiPassword_{SSID_PASSWORD};
  wifi_mode_t wifiMode_{WIFI_MODE_STA};
  std::unique_ptr<tcpip_adapter_ip_info_t> stationStaticIP_{nullptr};
  ip_addr_t stationDNSServer_{ip_addr_any};
};

extern unique_ptr<ConfigurationManager> configStore;
extern unique_ptr<Esp32WiFiManager> wifiManager;

#endif // CONFIG_MGR_H_