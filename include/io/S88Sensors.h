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

#ifndef S88_SENSORS_H_
#define S88_SENSORS_H_

#include "io/Sensors.h"

class S88Sensor : public Sensor {
public:
  S88Sensor(uint16_t, uint16_t);
  virtual ~S88Sensor() {}
  void check() {}
  void setState(bool state) {
    set(state);
  }
  void updateID(uint16_t newID) {
    setID(newID);
  }
  uint16_t getIndex() {
    return _index;
  }
private:
  uint16_t _index;
};

class S88SensorBus
{
public:
  S88SensorBus(const uint8_t, const uint8_t, const uint16_t);
  S88SensorBus(JsonObject &);
  void update(const uint8_t, const uint16_t);
  void toJson(JsonObject, bool=false);
  void addSensors(int16_t);
  void removeSensors(int16_t);
  std::string getStateString();
  uint8_t getID()
  {
    return _id;
  }
  uint8_t getDataPin()
  {
    return _dataPin;
  }
  uint16_t getSensorIDBase()
  {
    return _sensorIDBase;
  }
  uint16_t getSensorCount()
  {
    return _sensors.size();
  }
  void prepForRead()
  {
    _nextSensorToRead = 0;
  }
  bool hasMore()
  {
    return _nextSensorToRead < _sensors.size();
  }
  void readNext();
  void show();
private:
  uint8_t _id;
  uint8_t _dataPin;
  uint16_t _sensorIDBase;
  uint8_t _nextSensorToRead;
  uint16_t _lastSensorID;
  std::vector<S88Sensor *> _sensors;
};

class S88BusManager
{
public:
  static void init();
  static void clear();
  static uint8_t store();
  static void s88SensorTask(void *param);
  static bool createOrUpdateBus(const uint8_t, const uint8_t, const uint16_t);
  static bool removeBus(const uint8_t);
  static void getState(JsonArray);
private:
  static TaskHandle_t _taskHandle;
  static OSMutex _s88SensorLock;
};

DECLARE_DCC_PROTOCOL_COMMAND_CLASS(S88BusCommandAdapter, "S88")

#endif // S88_SENSORS_H_