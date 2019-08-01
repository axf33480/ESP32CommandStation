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

// This controls how often to send a speed update packet to the decoder,
// this is the minimum periodic refresh period. It will always be sent
// when the speed changes.
constexpr uint64_t LOCO_SPEED_PACKET_INTERVAL = MSEC_TO_USEC(100);

// This controls how often to send the locomotive function packets,
// default is approximately every second.
constexpr uint64_t LOCO_FUNCTION_PACKET_INTERVAL = SEC_TO_USEC(60);

static int8_t LOCO_REGISTER_ID = 1;

Locomotive::Locomotive(uint16_t address, bool managed) : Dcc128Train(DccLongAddress(address)), _registerNumber(LOCO_REGISTER_ID++)
{
  if (!managed)
  {
    packet_processor_remove_refresh_source(this);
  }
}

void Locomotive::showStatus()
{
  dcc::SpeedType speed(get_speed());
  LOG(INFO, "[Loco %d] speed: %d, direction: %s",
    legacy_address(), (speed.get_dcc_128() & 0x7F), speed.direction() ? JSON_VALUE_REVERSE : JSON_VALUE_FORWARD);
  wifiInterface.broadcast(StringPrintf("<T %d %d %d>", _registerNumber, speed.get_dcc_128(), speed.direction()));
}

void Locomotive::toJson(JsonObject jsonObject, bool includeSpeedDir, bool includeFunctions)
{
  jsonObject[JSON_ADDRESS_NODE] = legacy_address();
  if(includeSpeedDir)
  {
    dcc::SpeedType speed(get_speed());
    jsonObject[JSON_SPEED_NODE] = (speed.get_dcc_128() & 0x7F);
    jsonObject[JSON_DIRECTION_NODE] = speed.direction() ? JSON_VALUE_REVERSE : JSON_VALUE_FORWARD;
  }
  jsonObject[JSON_ORIENTATION_NODE] = _orientation ? JSON_VALUE_FORWARD : JSON_VALUE_REVERSE;
  if(includeFunctions)
  {
    JsonArray functions = jsonObject.createNestedArray(JSON_FUNCTIONS_NODE);
    for(uint8_t funcID = 0; funcID < p.get_max_fn(); funcID++)
    {
      JsonObject node = functions.createNestedObject();
      node[JSON_ID_NODE] = funcID;
      node[JSON_STATE_NODE] = get_fn(funcID) ? JSON_VALUE_TRUE : JSON_VALUE_FALSE;
    }
  }
}

Locomotive *Locomotive::fromJsonFile(const char *filename, bool managed)
{
  DynamicJsonDocument jsonBuffer{1024};
  JsonObject entry = configStore->load(filename, jsonBuffer);
  return fromJson(entry, managed);
}

Locomotive *Locomotive::fromJson(JsonObject json, bool managed)
{
  Locomotive *loco = new Locomotive(json[JSON_ADDRESS_NODE].is<uint16_t>(), managed);
  dcc::SpeedType speed(0);
  speed.set_dcc_128(json[JSON_SPEED_NODE].as<uint8_t>());
  speed.set_direction(json[JSON_DIRECTION_NODE] == JSON_VALUE_FORWARD);
  loco->set_speed(speed);
  loco->setOrientationForward(json[JSON_ORIENTATION_NODE] == JSON_VALUE_FORWARD);
  if (!json.getMember(JSON_FUNCTIONS_NODE).isNull())
  {
    for(JsonObject func : json.getMember(JSON_FUNCTIONS_NODE).as<JsonArray>())
    {
      loco->set_fn(func[JSON_ID_NODE], func[JSON_STATE_NODE] == JSON_VALUE_TRUE);
    }
  }
  return loco;
}