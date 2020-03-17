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

#ifndef S88_SENSORS_H_
#define S88_SENSORS_H_

#include <driver/gpio.h>

#include "Sensors.h"

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
  S88SensorBus(const uint8_t, const gpio_num_t, const uint16_t);
  S88SensorBus(std::string &);
  void update(const gpio_num_t, const uint16_t);
  std::string toJson(bool=false);
  void addSensors(int16_t);
  void removeSensors(int16_t);
  std::string getStateString();
  uint8_t getID()
  {
    return _id;
  }
  gpio_num_t getDataPin()
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
  std::string get_state_for_dccpp();
private:
  uint8_t _id;
  gpio_num_t _dataPin;
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
  static bool createOrUpdateBus(const uint8_t, const gpio_num_t, const uint16_t);
  static bool removeBus(const uint8_t);
  static std::string getStateAsJson();
  static std::string get_state_for_dccpp();
private:
  static OSMutex _s88SensorLock;
};

#endif // S88_SENSORS_H_