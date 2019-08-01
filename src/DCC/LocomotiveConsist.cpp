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

#include "ESP32CommandStation.h"

/**********************************************************************
ESP32 COMMAND STATION supports multiple Locomotive Consists, using either
command station consisting or decoder assisted consisting.

  <C ID LEAD TRAIL [{OTHER}]> : Creates a consist using ID with LEAD and TRAIL
                                locomotives with any OTHER locomotives in the
                                middle.
        NOTE: If the LOCO number is NEGATIVE it will be treated as being in
        reverse direction.
        NOTE: To create a decoder assisted consist specify the ID as a negative
        value or as ZERO.
        returns: <O> if successful and <X> if unsuccessful.
  <C ID LOCO>                 : Removes LOCO from consist ID when ID is not ZERO.
        returns: <O> if successful and <X> if unsuccessful.
  <C 0 LOCO>                  : Queries which consist LOCO is part of.
        returns: <V ID LOCO> with ID as ZERO if LOCO is not in a consist
        otherwise consist ID.
  <C ID>                      : Deletes definition of consist ID.
        returns: <O> if successful and <X> if unsuccessful.
  <C>                         : Lists all consists.
        returns: <U ID LEAD TRAIL [{OTHER}]> for each consist or <X> if no
        consists have been defined.

When either the leading or trailing locmotive is directly addressed all
locomotives in the consist will respond accordingly based on the configuration
of the consist. Therefore, the front light and rear lights of the lead and trail
locmotives will be active based on direction of travel and orientation of the
locomotive. As example:
CONSIST: 10
LEAD   : 4567 (FWD)
TRAIL  : 8765 (REV)
OTHER  : 1234 (REV), 2456 (REV)
When consist 10 moves in the FWD direction (in relation to lead loco) the
following functions will be active:
4567 FL
8765 NONE
1234 NONE
2456 NONE

When consist 10 moves in the REV direction (in relation to lead loco) the
following functions will be active:
4567 NONE
8765 FR
1234 NONE
2456 NONE

Configuration of advanced consist:
each locomotive will have the following CV values:
CV19 consist ID
CV21 ALL BITs ON
CV22 BIT 0 OFF, ALL others ON
CV29 BIT 0 set based on consist orientation.
CV29 BIT 1 set to ON (control FL via F0 only)

Configuration for command station managed consist:
NO CV changes, when consist is addressed (either by LEAD or TRAIL loco), all
locomotives in consist will be updated concurrently via multiple packet queuing.
**********************************************************************/

LocomotiveConsist::~LocomotiveConsist() {
  releaseLocomotives();
}

void LocomotiveConsist::showStatus() {
  // <U ID LEAD TRAIL [{OTHER}]>
  auto speed = get_speed();
  LOG(INFO, "[Consist %d] speed: %d, direction: %s, decoderAssisted: %s"
    , legacy_address(), (speed.get_dcc_128() & 0x7F)
    , speed.direction() ? JSON_VALUE_REVERSE : JSON_VALUE_FORWARD
    , _decoderAssisstedConsist ? JSON_VALUE_TRUE : JSON_VALUE_FALSE);
  string statusCmd = StringPrintf("<U %d", legacy_address() * _decoderAssisstedConsist ? -1 : 1);
  for (const auto& loco : _locos) {
    LOG(INFO, "LOCO: %d, ORIENTATION: %s", loco->legacy_address(),
      loco->isOrientationForward() ? JSON_VALUE_FORWARD : JSON_VALUE_REVERSE);
    statusCmd += StringPrintf(" %d", loco->legacy_address() * loco->isOrientationForward() ? 1 : -1);
  }
  statusCmd += ">";
  wifiInterface.broadcast(statusCmd);
}

void LocomotiveConsist::toJson(JsonObject jsonObject, bool includeSpeedDir, bool includeFunctions) {
  Locomotive::toJson(jsonObject, includeSpeedDir, includeFunctions);
  jsonObject[JSON_CONSIST_NODE] = JSON_VALUE_TRUE;
  jsonObject[JSON_DECODER_ASSISTED_NODE] = _decoderAssisstedConsist ? JSON_VALUE_TRUE : JSON_VALUE_FALSE;
  JsonArray locoArray = jsonObject.createNestedArray(JSON_LOCOS_NODE);
  for (const auto& loco : _locos) {
    loco->toJson(locoArray.createNestedObject(), includeSpeedDir, includeFunctions);
  }
}
LocomotiveConsist *LocomotiveConsist::fromJsonFile(const char *filename) {
  DynamicJsonDocument jsonBuffer{1024};
  JsonObject entry = configStore->load(filename, jsonBuffer);
  return fromJson(entry);
}

LocomotiveConsist *LocomotiveConsist::fromJson(JsonObject json) {
  LocomotiveConsist * consist =
    new LocomotiveConsist(json[JSON_ADDRESS_NODE].as<uint16_t>()
                        , json[JSON_DECODER_ASSISTED_NODE] == JSON_VALUE_TRUE);
  for(JsonObject member : json.getMember(JSON_LOCOS_NODE).as<JsonArray>())
  {
    if (member.getMember(JSON_FILE_NODE).isNull())
    {
      consist->_locos.push_back(Locomotive::fromJson(member, false));
    }
    else
    {
      consist->_locos.push_back(Locomotive::fromJsonFile(member.getMember(JSON_FILE_NODE).as<char *>(), false));
    }
  }
  return consist;
}

bool LocomotiveConsist::isAddressInConsist(uint16_t locoAddress) {
  for (const auto& loco : _locos) {
    if (loco->legacy_address() == locoAddress) {
      return true;
    }
  }
  return false;
}

void LocomotiveConsist::updateThrottle(uint16_t locoAddress, int8_t speed, bool forward)
{
  auto req_speed = get_speed();
  req_speed.set_dcc_128(speed);
  req_speed.set_direction(forward ? dcc::SpeedType::FORWARD : dcc::SpeedType::REVERSE);
  if (!_decoderAssisstedConsist)
  {
    // if it is a basic consist then sending a throttle request to any
    // locomotive in the consist will cause all locomotives to update
    for (const auto& loco : _locos)
    {
      loco->set_speed(req_speed);
    }
  }
  else if (_locos[0]->legacy_address() == locoAddress ||
           _locos[1]->legacy_address() == locoAddress ||
           legacy_address() == locoAddress)
  {
    // only if we are addressing the lead or trail locomotive should we react to
    // the throttle adjustment
    set_speed(req_speed);
  }
}

void LocomotiveConsist::addLocomotive(uint16_t locoAddress, bool forward,
  uint8_t position) {
  Locomotive *loco = LocomotiveManager::getLocomotive(locoAddress, false);
  loco->setOrientationForward(forward);
  _locos.push_back(loco);
  if(_decoderAssisstedConsist) {
    // write the loco consist address
    if(forward) {
      writeOpsCVByte(locoAddress, CV_NAMES::CONSIST_ADDRESS, legacy_address());
    } else {
      // if the locomotive is in reverse orientation set bit 7 on the consist
      // address to inform the decoder of this change
      // see s-9.2.2 CV 19 details
      writeOpsCVByte(locoAddress, CV_NAMES::CONSIST_ADDRESS,
        legacy_address() + CONSIST_ADDRESS_REVERSED_ORIENTATION);
    }
    // toggle FL/FR based on position, if it is the lead or trail locomotive
    // enable the function.
    if(position <= 1) {
      _locos[position]->set_fn(0, true);
      writeOpsCVBit(locoAddress, CV_NAMES::CONSIST_FUNCTION_CONTROL_FL_F9_F12,
        CONSIST_FUNCTION_CONTROL_FL_F9_F12_BITS::FL_BIT, false);
    } else {
      _locos[position]->set_fn(0, false);
      writeOpsCVBit(locoAddress, CV_NAMES::CONSIST_FUNCTION_CONTROL_FL_F9_F12,
        CONSIST_FUNCTION_CONTROL_FL_F9_F12_BITS::FL_BIT, true);
    }
    // enable F1-F8 for the consist address
    writeOpsCVByte(locoAddress, CV_NAMES::CONSIST_FUNCTION_CONTROL_F1_F8, 0xFF);
    // enable F9-F12 for the consist address
    writeOpsCVBit(locoAddress, CV_NAMES::CONSIST_FUNCTION_CONTROL_FL_F9_F12,
      CONSIST_FUNCTION_CONTROL_FL_F9_F12_BITS::F9_BIT, true);
    writeOpsCVBit(locoAddress, CV_NAMES::CONSIST_FUNCTION_CONTROL_FL_F9_F12,
      CONSIST_FUNCTION_CONTROL_FL_F9_F12_BITS::F10_BIT, true);
    writeOpsCVBit(locoAddress, CV_NAMES::CONSIST_FUNCTION_CONTROL_FL_F9_F12,
      CONSIST_FUNCTION_CONTROL_FL_F9_F12_BITS::F11_BIT, true);
    writeOpsCVBit(locoAddress, CV_NAMES::CONSIST_FUNCTION_CONTROL_FL_F9_F12,
      CONSIST_FUNCTION_CONTROL_FL_F9_F12_BITS::F12_BIT, true);
  }
}

bool LocomotiveConsist::removeLocomotive(uint16_t locoAddress) {
  uint8_t index = 0;
  bool locoFound = false;
  for(uint8_t index = 0; index < _locos.size(); index++) {
    if(_locos[index]->legacy_address() == locoAddress) {
      locoFound = true;
      break;
    }
  }
  if(locoFound) {
    _locos.erase(_locos.begin() + index);
    if(_decoderAssisstedConsist) {
      // if we are in an advanced consist, send a programming packet to clear
      // the consist address from the decoder
      writeOpsCVByte(locoAddress, CV_NAMES::CONSIST_ADDRESS, CONSIST_ADDRESS_NO_ADDRESS);
    }
  }
  return locoFound;
}

void LocomotiveConsist::releaseLocomotives() {
  for(uint8_t index = 0; index < _locos.size(); index++) {
    delete _locos[index];
  }
  _locos.clear();
}

void ConsistCommandAdapter::process(const vector<string> arguments) {
  if (arguments.empty()) {
    LocomotiveManager::showConsistStatus();
  } else if (arguments.size() == 1 &&
    LocomotiveManager::removeLocomotiveConsist(std::stoi(arguments[1]))) {
    wifiInterface.broadcast(COMMAND_SUCCESSFUL_RESPONSE);
  } else if (arguments.size() == 2) {
    int8_t consistAddress = std::stoi(arguments[0]);
    uint16_t locomotiveAddress = std::stoi(arguments[1]);
    if (consistAddress == 0) {
      // query which consist loco is in
      auto consist = LocomotiveManager::getConsistForLoco(locomotiveAddress);
      if (consist != nullptr) {
        wifiInterface.broadcast(StringPrintf("<V %d %d>",
          consist->legacy_address() * consist->isDecoderAssistedConsist() ? -1 : 1,
          locomotiveAddress));
        return;
      }
    } else {
      // remove loco from consist
      auto consist = LocomotiveManager::getConsistByID(consistAddress);
      if (consist->isAddressInConsist(locomotiveAddress)) {
        consist->removeLocomotive(locomotiveAddress);
        wifiInterface.broadcast(COMMAND_SUCCESSFUL_RESPONSE);
        return;
      }
    }
    // if we get here either the query or remove failed
    wifiInterface.broadcast(COMMAND_FAILED_RESPONSE);
  } else if (arguments.size() >= 3) {
    // create or update consist
    uint16_t consistAddress = std::stoi(arguments[0]);
    auto consist = LocomotiveManager::getConsistByID(consistAddress);
    if (consist != nullptr) {
      // existing consist, need to update
      consist->releaseLocomotives();
    } else {
      // verify if all provided locos are not already in a consist
      for(int index = 1; index < arguments.size(); index++) {
        int32_t locomotiveAddress = std::stoi(arguments[index]);
        if(LocomotiveManager::isAddressInConsist(abs(locomotiveAddress))) {
          LOG_ERROR("[Consist] Locomotive %d is already in a consist.", abs(locomotiveAddress));
          wifiInterface.broadcast(COMMAND_FAILED_RESPONSE);
          return;
        }
      }
      consist = LocomotiveManager::createLocomotiveConsist(consistAddress);
      if(consist == nullptr) {
        LOG_ERROR("[Consist] Unable to create new Consist");
        wifiInterface.broadcast(COMMAND_FAILED_RESPONSE);
        return;
      }
    }
    // add locomotives to consist
    for(int index = 1; index < arguments.size(); index++) {
      int32_t locomotiveAddress = std::stoi(arguments[index]);
      consist->addLocomotive(abs(locomotiveAddress), locomotiveAddress > 0,
        index - 1);
    }
  } else {
    wifiInterface.broadcast(COMMAND_FAILED_RESPONSE);
  }
}
