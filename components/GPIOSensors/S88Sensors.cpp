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

/**********************************************************************

The ESP32 Command Station supports multiple S88 Sensor busses.

To have the command station monitor an S88 sensor, first define/edit/delete
an S88 Sensor Bus using the following variations on the "S88" command:
  <S88 ID DATAPIN COUNT> : Creates an S88 Sensor Bus with the specified
                           ID, DATA PIN, SENSOR COUNT.
        returns: <O> if successful and <X> if unsuccessful.
  <S88 ID>               : Deletes definition of S88 Sensor Bus ID and all
                           associated S88 Sensors on the bus.
        returns: <O> if successful and <X> if unsuccessful.
  <S88>                  : Lists all S88 Sensor Busses and state of sensors.
        returns: <S88 ID DATAPIN> for each S88 Sensor Bus or <X>
        if no busses have been defined
Note: S88 Sensor Busses will create individual sensors that report via <S> but
they can not be edited/deleted via <S> commands. Attempts to do that will result
in an <X> being returned.

S88 Sensors are reported in the same manner as generic Sensors:
  <Q ID>     - for activation of S88 Sensor ID.
  <q ID>     - for deactivation of S88 Sensor ID.

**********************************************************************/
#if CONFIG_ENABLE_SENSORS && CONFIG_S88
#include <DCCppProtocol.h>
#include <StatusDisplay.h>
#include <json.hpp>
#include <JsonConstants.h>
#include "S88Sensors.h"

/////////////////////////////////////////////////////////////////////////////////////
// S88 Timing values (in microseconds)
/////////////////////////////////////////////////////////////////////////////////////
constexpr uint16_t S88_SENSOR_LOAD_PRE_CLOCK_TIME = 50;
constexpr uint16_t S88_SENSOR_LOAD_POST_RESET_TIME = 50;
constexpr uint16_t S88_SENSOR_CLOCK_PULSE_TIME = 50;
constexpr uint16_t S88_SENSOR_CLOCK_PRE_RESET_TIME = 50;
constexpr uint16_t S88_SENSOR_RESET_PULSE_TIME = 50;
constexpr uint16_t S88_SENSOR_READ_TIME = 25;

// This is the interval at which sensors will be checked
constexpr TickType_t S88_SENSOR_CHECK_DELAY = pdMS_TO_TICKS(50);

static constexpr const char * S88_SENSORS_JSON_FILE = "s88.json";

OSMutex S88BusManager::_s88SensorLock;

static constexpr UBaseType_t S88_SENSOR_TASK_PRIORITY = 1;
static constexpr uint32_t S88_SENSOR_TASK_STACK_SIZE = 2048;

std::vector<std::unique_ptr<S88SensorBus>> s88SensorBus;

void S88BusManager::init()
{
  LOG(INFO, "[S88] Configuration (clock: %d, reset: %d, load: %d)", CONFIG_S88_CLOCK_PIN, CONFIG_S88_RESET_PIN, CONFIG_S88_LOAD_PIN);
  gpio_pad_select_gpio((gpio_num_t)CONFIG_S88_CLOCK_PIN);
  ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)CONFIG_S88_CLOCK_PIN, GPIO_MODE_OUTPUT));
  gpio_pad_select_gpio((gpio_num_t)CONFIG_S88_RESET_PIN);
  ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)CONFIG_S88_RESET_PIN, GPIO_MODE_OUTPUT));
  gpio_pad_select_gpio((gpio_num_t)CONFIG_S88_LOAD_PIN);
  ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)CONFIG_S88_LOAD_PIN, GPIO_MODE_OUTPUT));

  LOG(INFO, "[S88] Initializing SensorBus list");
  nlohmann::json root = nlohmann::json::parse(Singleton<ConfigurationManager>::instance()->load(S88_SENSORS_JSON_FILE));
  uint16_t s88BusCount = root.contains(JSON_COUNT_NODE) ? root[JSON_COUNT_NODE].get<int>() : 0;
  Singleton<StatusDisplay>::instance()->status("Found %02d S88 Bus", s88BusCount);
  if(s88BusCount > 0)
  {
    for(auto bus : root[JSON_SENSORS_NODE])
    {
      string data = bus.dump();
      s88SensorBus.emplace_back(new S88SensorBus(data));
    }
  }
  LOG(INFO, "[S88] Loaded %d Sensor Buses", s88SensorBus.size());
  xTaskCreate(s88SensorTask, "S88SensorManager", S88_SENSOR_TASK_STACK_SIZE, NULL, S88_SENSOR_TASK_PRIORITY, nullptr);
}

void S88BusManager::clear()
{
  s88SensorBus.clear();
}

uint8_t S88BusManager::store()
{
  nlohmann::json root;
  uint8_t sensorBusIndex = 0;
  for (const auto& bus : s88SensorBus)
  {
    root[JSON_SENSORS_NODE].push_back(bus->toJson());
    sensorBusIndex++;
  }
  root[JSON_COUNT_NODE] = sensorBusIndex;
  Singleton<ConfigurationManager>::instance()->store(S88_SENSORS_JSON_FILE, root.dump());
  return sensorBusIndex;
}

void S88BusManager::s88SensorTask(void *param)
{
  while(true)
  {
    OSMutexLock l(&_s88SensorLock);
    for (const auto& sensorBus : s88SensorBus)
    {
      sensorBus->prepForRead();
    }
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)CONFIG_S88_LOAD_PIN, 1));
    delayMicroseconds(S88_SENSOR_LOAD_PRE_CLOCK_TIME);
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)CONFIG_S88_CLOCK_PIN, 1));
    delayMicroseconds(S88_SENSOR_CLOCK_PULSE_TIME);
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)CONFIG_S88_CLOCK_PIN, 0));
    delayMicroseconds(S88_SENSOR_CLOCK_PRE_RESET_TIME);
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)CONFIG_S88_RESET_PIN, 1));
    delayMicroseconds(S88_SENSOR_RESET_PULSE_TIME);
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)CONFIG_S88_RESET_PIN, 0));
    delayMicroseconds(S88_SENSOR_LOAD_POST_RESET_TIME);
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)CONFIG_S88_LOAD_PIN, 0));

    delayMicroseconds(S88_SENSOR_READ_TIME);
    bool keepReading = true;
    while(keepReading)
    {
      keepReading = false;
      for (const auto& sensorBus : s88SensorBus)
      {
        if(sensorBus->hasMore())
        {
          keepReading = true;
          sensorBus->readNext();
        }
      }
      ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)CONFIG_S88_CLOCK_PIN, 1));
      delayMicroseconds(S88_SENSOR_CLOCK_PULSE_TIME);
      ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)CONFIG_S88_CLOCK_PIN, 0));
      delayMicroseconds(S88_SENSOR_READ_TIME);
    }
    vTaskDelay(S88_SENSOR_CHECK_DELAY);
  }
}

bool S88BusManager::createOrUpdateBus(const uint8_t id, const uint8_t dataPin, const uint16_t sensorCount)
{
  // check for duplicate data pin
  for (const auto& sensorBus : s88SensorBus)
  {
    if(sensorBus->getID() != id && sensorBus->getDataPin() == dataPin)
    {
      LOG_ERROR("[S88] Bus %d is already using data pin %d, rejecting create/update of S88 Bus %d",
        sensorBus->getID(), dataPin, id);
      return false;
    }
  }
  OSMutexLock l(&_s88SensorLock);
  // check for existing bus to be updated
  for (const auto& sensorBus : s88SensorBus)
  {
    if(sensorBus->getID() == id)
    {
      sensorBus->update(dataPin, sensorCount);
      return true;
    }
  }
  if(is_restricted_pin(dataPin))
  {
    LOG_ERROR("[S88] Attempt to use a restricted pin: %d", dataPin);
    return false;
  }
  s88SensorBus.push_back(std::make_unique<S88SensorBus>(id, dataPin, sensorCount));
  return true;
}

bool S88BusManager::removeBus(const uint8_t id)
{
  OSMutexLock l(&_s88SensorLock);
  const auto & ent = std::find_if(s88SensorBus.begin(), s88SensorBus.end(),
  [id](std::unique_ptr<S88SensorBus> & bus) -> bool
  {
    return bus->getID() == id;
  });
  if (ent != s88SensorBus.end())
  {
    s88SensorBus.erase(ent);
    return true;
  }
  return false;
}

string S88BusManager::getStateAsJson()
{
  string state = "[";
  for (const auto& sensorBus : s88SensorBus)
  {
    state += sensorBus->toJson(true);
    state += ",";
  }
  state += "]";
  return state;
}

string S88BusManager::get_state_for_dccpp()
{
  string res;
  for (const auto& sensorBus : s88SensorBus)
  {
    res += sensorBus->get_state_for_dccpp();
  }
  return res;
}

S88SensorBus::S88SensorBus(const uint8_t id, const uint8_t dataPin, const uint16_t sensorCount) :
  _id(id), _dataPin(dataPin), _sensorIDBase((id * S88_MAX_SENSORS_PER_BUS) + S88_FIRST_SENSOR),
  _lastSensorID((id * S88_MAX_SENSORS_PER_BUS) + S88_FIRST_SENSOR)
{
  LOG(INFO, "[S88 Bus-%d] Created using data pin %d with %d sensors starting at id %d",
    _id, _dataPin, sensorCount, _sensorIDBase);
  gpio_pad_select_gpio((gpio_num_t)_dataPin);
  ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)_dataPin, GPIO_MODE_INPUT));
  if(sensorCount > 0)
  {
    addSensors(sensorCount);
  }
}

S88SensorBus::S88SensorBus(string &data)
{
  nlohmann::json object = nlohmann::json::parse(data);
  _id = object[JSON_ID_NODE];
  _dataPin = object[JSON_PIN_NODE];
  _lastSensorID = _sensorIDBase = object[JSON_S88_SENSOR_BASE_NODE];
  uint16_t sensorCount = object[JSON_COUNT_NODE];
  Singleton<StatusDisplay>::instance()->status("S88(%d) %02d sensors", _id
  , sensorCount);
  if(sensorCount > 0)
  {
    addSensors(sensorCount);
  }
}

void S88SensorBus::update(const uint8_t dataPin, const uint16_t sensorCount)
{
  _dataPin = dataPin;
  _lastSensorID = _sensorIDBase;
  gpio_pad_select_gpio((gpio_num_t)_dataPin);
  ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)_dataPin, GPIO_MODE_INPUT));  
  for (const auto& sensor : _sensors)
  {
    sensor->updateID(_lastSensorID++);
  }
  if(sensorCount < _sensors.size())
  {
    removeSensors(_sensors.size() - sensorCount);
  }
  else if(sensorCount > 0)
  {
    addSensors(sensorCount - _sensors.size());
  }
  LOG(INFO, "[S88 Bus-%d] Updated to use data pin %d with %d sensors",
    _id, _dataPin, _sensors.size());
}

string S88SensorBus::toJson(bool includeState)
{
  nlohmann::json object =
  {
    { JSON_ID_NODE, _id },
    { JSON_PIN_NODE, _dataPin },
    { JSON_S88_SENSOR_BASE_NODE, _sensorIDBase },
    { JSON_COUNT_NODE, _sensors.size() },
  };
  if(includeState)
  {
    object[JSON_STATE_NODE] = getStateString();
  }
  return object.dump();
}

void S88SensorBus::addSensors(int16_t sensorCount)
{
  const uint16_t startingIndex = _sensors.size();
  for(uint8_t id = 0; id < sensorCount; id++)
  {
    _sensors.push_back(new S88Sensor(_lastSensorID++, startingIndex + id));
  }
}

void S88SensorBus::removeSensors(int16_t sensorCount)
{
  if(sensorCount < 0)
  {
    for (const auto& sensor : _sensors)
    {
      LOG(VERBOSE, "[S88] Sensor(%d) removed", sensor->getID());
    }
    _sensors.clear();
  }
  else
  {
    for(uint8_t id = 0; id < sensorCount; id++)
    {
      LOG(VERBOSE, "[S88] Sensor(%d) removed", _sensors.back()->getID());
      _sensors.pop_back();
    }
  }
}

string S88SensorBus::getStateString()
{
  string state = "";
  for (const auto& sensor : _sensors)
  {
    if(sensor->isActive())
    {
      state += "1";
    }
    else
    {
      state += "0";
    }
  }
  return state;
}

void S88SensorBus::readNext()
{
  // sensors need to pull pin LOW for ACTIVE
  _sensors[_nextSensorToRead++]->setState(digitalRead(_dataPin) == HIGH);
}

string S88SensorBus::get_state_for_dccpp()
{
  string status = StringPrintf("<S88 %d %d %d>", _id, _dataPin, _sensors.size());
  LOG(VERBOSE, "[S88 Bus-%d] Data:%d, Base:%d, Count:%d:", _id, _dataPin, _sensorIDBase, _sensors.size());
  for (const auto& sensor : _sensors)
  {
    LOG(VERBOSE, "[S88] Input: %d :: %s", sensor->getIndex(), sensor->isActive() ? "ACTIVE" : "INACTIVE");
    status += sensor->get_state_for_dccpp();
  }
  return status;
}

S88Sensor::S88Sensor(uint16_t id, uint16_t index) : Sensor(id, NON_STORED_SENSOR_PIN, false, false), _index(index)
{
  LOG(VERBOSE, "[S88] Sensor(%d) created with index %d", id, _index);
}

DCC_PROTOCOL_COMMAND_HANDLER(S88BusCommandAdapter,
[](const vector<string> arguments)
{
  if(arguments.empty())
  {
    // list all sensor groups
    return S88BusManager::get_state_for_dccpp();
  }
  else
  {
    if (arguments.size() == 1 &&
        S88BusManager::removeBus(std::stoi(arguments[0])))
    {
      // delete sensor bus
      return COMMAND_SUCCESSFUL_RESPONSE;
    }
    else if (arguments.size() == 3 &&
             S88BusManager::createOrUpdateBus(std::stoi(arguments[0])
                                            , std::stoi(arguments[1])
                                            , std::stoi(arguments[2])))
    {
      // create sensor bus
      return COMMAND_SUCCESSFUL_RESPONSE;
    }
  }
  return COMMAND_FAILED_RESPONSE;
})

#endif // CONFIG_ENABLE_SENSORS