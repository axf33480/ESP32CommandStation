/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2018 Dan Worth
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

#include "ESP32CommandStation.h"

/**********************************************************************

The ESP32 Command Station supports remote sensor inputs that are connected via a
WiFi connection. Remote Sensors are dynamically created during startup or by a
remote sensor reporting its state.

During startup, the command station scans for Access Points that have a name
starting with REMOTE_SENSORS_PREFIX defined in Config.h, ie: "sensor01". If no
Access Points are found matching this prefix during startup they will be created
automatically when the sensor reports its state to the command station.

Note: Remote Sensors should not maintain a persistent connection. Instead they
should connect when a change occurs that should be reported. It is not necessary
for Remote Sensors to report when they are INACTIVE. If a Remote Sensor does not
report within REMOTE_SENSORS_DECAY milliseconds the command station will
automatically transition the Remote Sensor to INACTIVE state if it was
previously ACTIVE.

The following varations of the "RS" command :

  <RS ID STATE>:      Informs the command station of the status of a remote sensor.
  <RS ID>:            Deletes remote sensor ID.
  <RS>:               Lists all defined remote sensors.
                      returns: <RS ID STATE> for each defined remote sensor or
                      <X> if no remote sensors have been defined/found.
where

  ID:     the numeric ID (0-32667) of the remote sensor.
  STATE:  State of the sensors, zero is INACTIVE, non-zero is ACTIVE.
          Usage is remote sensor dependent.
**********************************************************************/

#if ENABLE_SENSORS
// TODO: merge this into the base SensorManager code.

// sanity check to ensure configuration has been setup correctly, default
// any missing parameters
#ifndef REMOTE_SENSORS_PREFIX
#define REMOTE_SENSORS_PREFIX "sensor"
#endif
#ifndef REMOTE_SENSORS_DECAY
#define REMOTE_SENSORS_DECAY 60000
#endif
#ifndef REMOTE_SENSORS_FIRST_SENSOR
#define REMOTE_SENSORS_FIRST_SENSOR 100
#endif
#ifndef SCAN_REMOTE_SENSORS_ON_STARTUP
#define SCAN_REMOTE_SENSORS_ON_STARTUP false
#endif

vector<unique_ptr<RemoteSensor>> remoteSensors;

void RemoteSensorManager::init()
{
#if SCAN_REMOTE_SENSORS_ON_STARTUP
  Singleton<InfoScreen>::instance()->replaceLine(
    INFO_SCREEN_ROTATING_STATUS_LINE, "WiFiScan started");
  int8_t networksFound;

  LOG(VERBOSE, "[RemoteSensors] Scanning for RemoteSensors");
  WiFi.scanNetworks(true);
  while((networksFound = WiFi.scanComplete()) < 0)
  {
    delay(100);
    LOG(INFO, ".");
    Singleton<InfoScreen>::instance()->replaceLine(
      INFO_SCREEN_ROTATING_STATUS_LINE, "WiFiScan pending");
  }
  const uint8_t REMOTE_SENSOR_PREFIX_LEN = String(REMOTE_SENSORS_PREFIX).length();
  for (int8_t i = 0; i < networksFound; i++)
  {
    if(WiFi.SSID(i).startsWith(REMOTE_SENSORS_PREFIX))
    {
      const uint16_t sensorID = String(WiFi.SSID(i)).substring(REMOTE_SENSOR_PREFIX_LEN).toInt();
      LOG(VERBOSE, "[RemoteSensors] Found %s, assigning as sensor %d", WiFi.SSID(i).c_str(), sensorID);
      Singleton<InfoScreen>::instance()->replaceLine(
        INFO_SCREEN_ROTATING_STATUS_LINE, F("RS %d: %s"), sensorID, WiFi.SSID(i).c_str());
      createOrUpdate(sensorID);
    }
  }
#else
  LOG(VERBOSE, "[RemoteSensors] Scanning for RemoteSensors DISABLED, remote sensors will only be created after reporting state");
#endif
}

void RemoteSensorManager::createOrUpdate(const uint16_t id, const uint16_t value) {
  // check for duplicate ID
  for (const auto& sensor : remoteSensors)
  {
    if(sensor->getRawID() == id)
    {
      sensor->setSensorValue(value);
      return;
    }
  }
  remoteSensors.emplace_back(new RemoteSensor(id, value));
}

bool RemoteSensorManager::remove(const uint16_t id)
{
  auto ent = std::find_if(remoteSensors.begin(), remoteSensors.end(),
  [id](unique_ptr<RemoteSensor> & sensor) -> bool
  {
    return sensor->getID() == id;
  });
  if (ent != remoteSensors.end())
  {
    remoteSensors.erase(ent);
    return true;
  }
  return false;
}

string RemoteSensorManager::getStateAsJson()
{
  string output = "[";
  for (const auto& sensor : remoteSensors)
  {
    output += sensor->toJson();
    output += ",";
  }
  output += "]";
  return output;
}

string RemoteSensorManager::get_state_for_dccpp()
{
  if (remoteSensors.empty())
  {
    return COMMAND_FAILED_RESPONSE;
  }
  string status;
  for (const auto& sensor : remoteSensors)
  {
    status += sensor->get_state_for_dccpp();
  }
  return status;
}

RemoteSensor::RemoteSensor(uint16_t id, uint16_t value) :
  Sensor(id + REMOTE_SENSORS_FIRST_SENSOR, NON_STORED_SENSOR_PIN, false, false), _rawID(id)
{
  setSensorValue(value);
  LOG(VERBOSE, "[RemoteSensors] RemoteSensor(%d) created with Sensor(%d), active: %s, value: %d",
    getRawID(), getID(), isActive() ? JSON_VALUE_TRUE : JSON_VALUE_FALSE, value);
}

void RemoteSensor::check()
{
  if(isActive() && millis() > _lastUpdate + REMOTE_SENSORS_DECAY)
  {
    LOG(INFO, "[RemoteSensors] RemoteSensor(%d) expired, deactivating", getRawID());
    setSensorValue(0);
  }
}

string RemoteSensor::get_state_for_dccpp()
{
  return StringPrintf("<RS %d %d>", getRawID(), _value);
}

string RemoteSensor::toJson(bool includeState)
{
  nlohmann::json object =
  {
    { JSON_ID_NODE, getRawID() },
    { JSON_VALUE_NODE, getSensorValue() },
    { JSON_STATE_NODE, isActive() },
    { JSON_LAST_UPDATE_NODE, getLastUpdate() },
    { JSON_PIN_NODE, getPin() },
    { JSON_PULLUP_NODE, isPullUp() },
  };
  return object.dump();
}

#endif // ENABLE_SENSORS