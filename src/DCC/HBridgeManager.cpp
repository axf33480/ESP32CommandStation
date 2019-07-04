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

#include "ESP32CommandStation.h"

vector<MonitoredHBridge *> monitoredHBridges;

void register_monitored_hbridge(SimpleCanStack *stack
                              , const adc1_channel_t senseChannel
                              , const gpio_num_t enablePin
                              , const gpio_num_t thermalWarningPin
                              , const uint32_t limitMilliAmps
                              , const uint32_t maxMilliAmps
                              , const string &name
                              , const string &bridgeType
                              , const TrackOutputConfig &cfg
                              , const bool programmingTrack)
{
  monitoredHBridges.push_back(
    new MonitoredHBridge(stack
                        , senseChannel
                        , enablePin
                        , thermalWarningPin
                        , limitMilliAmps
                        , maxMilliAmps
                        , name
                        , bridgeType
                        , cfg
                        , programmingTrack)
  );
}

void get_hbridge_status_json(JsonArray output)
{
  for (auto hbridge : monitoredHBridges)
  {
    output.add(serialized(hbridge->getStateAsJson()));
  }
}

bool is_track_power_on()
{
  for (auto hbridge : monitoredHBridges)
  {
    if (hbridge->isEnabled())
    {
      return true;
    }
  }
  return false;
}

void enable_all_hbridges()
{
  for (auto hbridge : monitoredHBridges)
  {
    if (!hbridge->isProgrammingTrack())
    {
      hbridge->enable();
    }
  }
}

void enable_named_hbridge(string name)
{
  for (auto hbridge : monitoredHBridges)
  {
    if (hbridge->getName() == name && !hbridge->isProgrammingTrack())
    {
      hbridge->enable();
      return;
    }
  }
}

void disable_all_hbridges()
{
  for (auto hbridge : monitoredHBridges)
  {
    hbridge->disable();
  }
}

void disable_named_hbridge(string name)
{
  for (auto hbridge : monitoredHBridges)
  {
    if (hbridge->getName() == name)
    {
      hbridge->disable();
      return;
    }
  }
}

uint32_t get_hbridge_sample(string name)
{
  for (auto hbridge : monitoredHBridges)
  {
    if (hbridge->getName() == name)
    {
      return hbridge->getLastReading();
    }
  }
  return 0;
}

void broadcast_all_hbridge_statuses()
{
  for (auto hbridge : monitoredHBridges)
  {
    hbridge->broadcastStatus();
  }
}

void broadcast_named_hbridge_status(string name)
{
  for (auto hbridge : monitoredHBridges)
  {
    if (hbridge->getName() == name)
    {
      hbridge->broadcastStatus();
    }
  }
}

string get_hbridge_info_screen_data()
{
  static uint8_t _hbIndex = 0;
  _hbIndex++;
  if (_hbIndex >= monitoredHBridges.size())
  {
    _hbIndex = 0;
  }
  return monitoredHBridges[_hbIndex]->getInfoScreenData();
}