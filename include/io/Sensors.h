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

#include "ESP32CommandStation.h"

const int8_t NON_STORED_SENSOR_PIN=-1;

class Sensor {
public:
  Sensor(uint16_t, int8_t, bool=false, bool=true);
  Sensor(JsonObject);
  virtual ~Sensor() {}
  void update(uint8_t, bool=false);
  virtual void toJson(JsonObject, bool=false);
  uint16_t getID() {
    return _sensorID;
  }
  int8_t getPin() {
    return _pin;
  }
  bool isPullUp() {
    return _pullUp;
  }
  bool isActive() {
    return _lastState;
  }
  virtual void check();
  void show();
protected:
  void set(bool state) {
    if(_lastState != state) {
      _lastState = state;
      LOG(INFO, "Sensor: %d :: %s", _sensorID, _lastState ? "ACTIVE" : "INACTIVE");
      wifiInterface.broadcast(StringPrintf("<%c %d>", state ? "Q" : "q", _sensorID));
    }
  }
  void setID(uint16_t id) {
    _sensorID = id;
  }
private:
  uint16_t _sensorID;
  int8_t _pin;
  bool _pullUp;
  bool _lastState;
};

class SensorManager {
public:
  static void init();
  static void clear();
  static uint16_t store();
  static void sensorTask(void *param);
  static void getState(JsonArray);
  static Sensor *getSensor(uint16_t);
  static bool createOrUpdate(const uint16_t, const uint8_t, const bool);
  static bool remove(const uint16_t);
  static int8_t getSensorPin(const uint16_t);
private:
  static TaskHandle_t _taskHandle;
  static xSemaphoreHandle _lock;
};

class SensorCommandAdapter : public DCCPPProtocolCommand {
public:
  void process(const std::vector<std::string>);
  std::string getID() {
    return "S";
  }
};
