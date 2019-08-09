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

static int8_t LOCO_REGISTER_ID = 1;

Locomotive::Locomotive(uint16_t address, TrainService *trainService)
  : Dcc128Train(DccLongAddress(address))
  , TrainNodeForProxy(trainService, this)
  , _registerNumber(LOCO_REGISTER_ID++)
{
  LOG(INFO, "[Loco %d] Created", address);
}

string Locomotive::getStateAsDCCpp()
{
  dcc::SpeedType speed(get_speed());
  LOG(INFO, "[Loco %d] speed: %d, direction: %s",
    legacy_address(), (speed.get_dcc_128() & 0x7F), speed.direction() ? JSON_VALUE_REVERSE : JSON_VALUE_FORWARD);
  return StringPrintf("<T %d %d %d>", _registerNumber, speed.get_dcc_128(), speed.direction());
}

string Locomotive::toJson(bool includeFunctions)
{
  json object;
  object[JSON_ADDRESS_NODE] = legacy_address();
  dcc::SpeedType speed(get_speed());
  object[JSON_SPEED_NODE] = (speed.get_dcc_128() & 0x7F);
  object[JSON_DIRECTION_NODE] = speed.direction() ? JSON_VALUE_REVERSE : JSON_VALUE_FORWARD;
  object[JSON_ORIENTATION_NODE] = _orientation ? JSON_VALUE_FORWARD : JSON_VALUE_REVERSE;
  if(includeFunctions)
  {
    for(uint8_t funcID = 0; funcID < p.get_max_fn(); funcID++)
    {
      object[JSON_FUNCTIONS_NODE].push_back({
        { JSON_ID_NODE, funcID },
        { JSON_STATE_NODE, get_fn(funcID) }
      });
    }
  }
  return object.dump();
}

Locomotive *Locomotive::fromJson(string &content, TrainService *trainService)
{
  json object = json::parse(content);
  Locomotive *loco = new Locomotive(object[JSON_ADDRESS_NODE].get<uint16_t>(), trainService);
  dcc::SpeedType speed(0);
  speed.set_dcc_128(object[JSON_SPEED_NODE].get<uint8_t>());
  speed.set_direction(object[JSON_DIRECTION_NODE] == JSON_VALUE_FORWARD);
  loco->set_speed(speed);
  loco->setOrientationForward(object[JSON_ORIENTATION_NODE] == JSON_VALUE_FORWARD);
  if (!object[JSON_FUNCTIONS_NODE].is_null())
  {
    for(auto func : object[JSON_FUNCTIONS_NODE])
    {
      loco->set_fn(func[JSON_ID_NODE].get<int>()
                , !func[JSON_STATE_NODE].get<string>().compare(JSON_VALUE_TRUE));
    }
  }
  return loco;
}