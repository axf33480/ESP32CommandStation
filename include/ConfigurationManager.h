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

#pragma once

#include <ArduinoJson.h>
#include <openlcb/Defs.hxx>

// Class definition for the Configuration Management system in ESP32 Command Station
class ConfigurationManager {
public:
  ConfigurationManager();
  virtual ~ConfigurationManager();
  void init();
  void clear();

  bool exists(const char *);
  void remove(const char *);
  JsonObject &load(const char *);
  JsonObject &load(const char *, DynamicJsonBuffer &);
  void store(const char *, const JsonObject &);
  JsonObject &createRootNode(bool=true);
  openlcb::NodeID getNodeId();
  bool needLCCCan(gpio_num_t *rxPin, gpio_num_t *txPin);
  void configureWiFi(openlcb::SimpleCanStack *stack, const WiFiConfiguration &cfg);
};

extern ConfigurationManager *configStore;
