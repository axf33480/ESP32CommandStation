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

using nlohmann::json;

unique_ptr<TurnoutManager> turnoutManager;

static constexpr const char * TURNOUTS_JSON_FILE = "turnouts.json";

static constexpr const char *TURNOUT_TYPE_STRINGS[] =
{
  "LEFT",
  "RIGHT",
  "WYE",
  "MULTI"
};

TurnoutManager::TurnoutManager(openlcb::Node *node)
{
  LOG(INFO, "[Turnout] Initializing DCC Turnout database");
  json root = json::parse(configStore->load(TURNOUTS_JSON_FILE));
  if (root.contains(JSON_COUNT_NODE) && root[JSON_COUNT_NODE].get<uint16_t>() > 0)
  {
    uint16_t turnoutCount = root[JSON_COUNT_NODE].get<uint16_t>();
    Singleton<InfoScreen>::instance()->replaceLine(
      INFO_SCREEN_ROTATING_STATUS_LINE, "Found %02d Turnouts", turnoutCount);
    for (auto turnout : root[JSON_TURNOUTS_NODE])
    {
      string data = turnout.dump();
      turnouts_.emplace_back(new Turnout(data));
    }
  }
  LOG(INFO, "[Turnout] Found %d DCC turnout(s)", turnouts_.size());

  // register the LCC event handler
  turnoutEventConsumer_.reset(new DccAccyConsumer(node, this));
}

void TurnoutManager::clear()
{
  AtomicHolder h(this);
  for (auto & turnout : turnouts_)
  {
    turnout.reset(nullptr);
  }
  turnouts_.clear();
}

uint16_t TurnoutManager::store()
{
  json root;
  uint16_t turnoutStoredCount = 0;
  for (const auto& turnout : turnouts_)
  {
    root.push_back(turnout->toJson());
    turnoutStoredCount++;
  }
  root[JSON_COUNT_NODE] = turnoutStoredCount;
  configStore->store(TURNOUTS_JSON_FILE, root);
  return turnoutStoredCount;
}

string TurnoutManager::setByID(uint16_t id, bool thrown, bool sendDCC)
{
  AtomicHolder h(this);
  for (auto & turnout : turnouts_)
  {
    if (turnout->getID() == id)
    {
      turnout->set(thrown, sendDCC);
      return turnout->getStateAsDCCpp();
    }
  }
  return COMMAND_FAILED_RESPONSE;
}

std::string TurnoutManager::setByAddress(uint16_t address, bool thrown, bool sendDCC)
{
  AtomicHolder h(this);
  for (auto & turnout : turnouts_)
  {
    if (turnout->getAddress() == address)
    {
      turnout->set(thrown, sendDCC);
      return turnout->getStateAsDCCpp();
    }
  }

  // we didn't find it, create it and set it
  turnouts_.emplace_back(new Turnout(turnouts_.size() + 1, address, -1));
  return setByAddress(address, thrown, sendDCC);
}

std::string TurnoutManager::toggleByID(uint16_t id)
{
  AtomicHolder h(this);
  for (auto & turnout : turnouts_)
  {
    if (turnout->getID() == id)
    {
      turnout->toggle();
      return turnout->getStateAsDCCpp();
    }
  }
  return COMMAND_FAILED_RESPONSE;
}

std::string TurnoutManager::toggleByAddress(uint16_t address)
{
  AtomicHolder h(this);
  auto const &elem = std::find_if(turnouts_.begin(), turnouts_.end(),
    [address](unique_ptr<Turnout> & turnout) -> bool
    {
      return (turnout->getAddress() == address);
    }
  );
  if (elem != turnouts_.end())
  {
    elem->get()->toggle();
    return elem->get()->getStateAsDCCpp();
  }

  // we didn't find it, create it and throw it
  turnouts_.emplace_back(new Turnout(turnouts_.size() + 1, address, -1));
  return toggleByAddress(address);
}

string TurnoutManager::getStateAsJson(bool readableStrings)
{
  json root;
  AtomicHolder h(this);
  for (auto& turnout : turnouts_)
  {
    root.push_back(turnout->toJson(readableStrings));
  }
  return root.dump();
}

string TurnoutManager::getStateAsDCCpp()
{
  AtomicHolder h(this);
  string status;
  for (auto& turnout : turnouts_)
  {
    status += turnout->getStateAsDCCpp();
  }
  return status;
}

Turnout *TurnoutManager::createOrUpdate(const uint16_t id, const uint16_t address, const int8_t index, const TurnoutType type)
{
  AtomicHolder h(this);
  auto const &elem = std::find_if(turnouts_.begin(), turnouts_.end(),
    [id](unique_ptr<Turnout> & turnout) -> bool
    {
      return (turnout->getID() == id);
    }
  );
  if (elem != turnouts_.end())
  {
    elem->get()->update(address, index, type);
    return elem->get();
  }
  // we didn't find it, create it!
  turnouts_.emplace_back(new Turnout(id, address, index, false, type));
  return turnouts_.back().get();
}

bool TurnoutManager::removeByID(const uint16_t id)
{
  AtomicHolder h(this);
  auto const &elem = std::find_if(turnouts_.begin(), turnouts_.end(),
    [id](unique_ptr<Turnout> & turnout) -> bool
    {
      return (turnout->getID() == id);
    }
  );
  if (elem != turnouts_.end())
  {
    LOG(VERBOSE, "[Turnout %d] Deleted", elem->get()->getID());
    turnouts_.erase(elem);
    return true;
  }
  return false;
}

bool TurnoutManager::removeByAddress(const uint16_t address)
{
  AtomicHolder h(this);
  auto const &elem = std::find_if(turnouts_.begin(), turnouts_.end(),
    [address](unique_ptr<Turnout> & turnout) -> bool
    {
      return (turnout->getAddress() == address);
    }
  );
  if (elem != turnouts_.end())
  {
    LOG(VERBOSE, "[Turnout %d] Deleted as it used address %d", elem->get()->getID(), address);
    turnouts_.erase(elem);
    return true;
  }
  return false;
}

Turnout *TurnoutManager::getTurnoutByIndex(const uint16_t index)
{
  AtomicHolder h(this);
  if (index < turnouts_.size())
  {
    return turnouts_[index].get();
  }
  return nullptr;
}

Turnout *TurnoutManager::getTurnoutByID(const uint16_t id)
{
  AtomicHolder h(this);
  auto const &elem = std::find_if(turnouts_.begin(), turnouts_.end(),
    [id](unique_ptr<Turnout> & turnout) -> bool {
      return (turnout->getID() == id);
    }
  );
  if (elem != turnouts_.end()) {
    return elem->get();
  }
  return nullptr;
}

Turnout *TurnoutManager::getTurnoutByAddress(const uint16_t address)
{
  AtomicHolder h(this);
  auto const &elem = std::find_if(turnouts_.begin(), turnouts_.end(),
    [address](unique_ptr<Turnout> & turnout) -> bool {
      return (turnout->getAddress() == address);
    }
  );
  if (elem != turnouts_.end())
  {
    return elem->get();
  }
  return nullptr;
}

uint16_t TurnoutManager::getTurnoutCount()
{
  AtomicHolder h(this);
  return turnouts_.size();
}

// TODO shift this to consume the LCC event directly
void TurnoutManager::send(Buffer<dcc::Packet> *b, unsigned prio)
{
  // add ref count so send doesn't delete it
  dcc::Packet *pkt = b->data();
  // Verify that the packet looks like a DCC Accessory decoder packet
  if(!pkt->packet_header.is_marklin &&
      pkt->dlc == 2 &&
      pkt->payload[0] & 0x80 &&
      pkt->payload[1] & 0x80)
  {
    // the second byte of the payload contains part of the address and is
    // stored in ones complement format.
    uint8_t onesComplementByteTwo = (pkt->payload[1] ^ 0xF8);
    // decode the accessories decoder address from the packet payload.
    uint16_t boardAddress = (pkt->payload[0] & 0x3F) +
                            ((onesComplementByteTwo >> 4) & 0x07);
    uint8_t boardIndex = ((onesComplementByteTwo >> 1) % 4);
    bool state = onesComplementByteTwo & 0x01;
    // Set the turnout to the requested state, don't send a DCC packet.
    setByAddress(decodeDCCAccessoryAddress(boardAddress, boardIndex), state);
  }
  b->unref();
}

void encodeDCCAccessoryAddress(uint16_t *boardAddress, int8_t *boardIndex, uint16_t address)
{
  *boardAddress = (address + 3) / 4;
  *boardIndex = (address - (*boardAddress * 4)) + 3;
}

uint16_t decodeDCCAccessoryAddress(uint16_t boardAddress, int8_t boardIndex)
{
  return (boardAddress * 4 + boardIndex) - 3;
}

Turnout::Turnout(uint16_t turnoutID
               , uint16_t address
               , int8_t index
               , bool thrown
               , TurnoutType type) :
                 _turnoutID(turnoutID)
               , _address(address)
               , _index(index)
               , _boardAddress(0)
               , _thrown(thrown)
               , _type(type)
{
  if (index == -1)
  {
    // convert the provided decoder address to a board address and accessory index
    encodeDCCAccessoryAddress(&_boardAddress, &_index, _address);
    LOG(INFO
      , "[Turnout %d] Created using DCC address %d as type %s and initial state of %s"
      , _turnoutID
      , _address
      , TURNOUT_TYPE_STRINGS[_type]
      , _thrown ? JSON_VALUE_THROWN : JSON_VALUE_CLOSED);
  }
  else
  {
    LOG(INFO
      , "[Turnout %d] Created using address %d:%d as type %s and initial state of %s"
      , _turnoutID
      , _address
      , _index
      , TURNOUT_TYPE_STRINGS[_type]
      , _thrown ? JSON_VALUE_THROWN : JSON_VALUE_CLOSED);
  }
}

Turnout::Turnout(string &data)
{
  json object = json::parse(data);
  _turnoutID = object[JSON_ID_NODE].get<int>();
  _address = object[JSON_ADDRESS_NODE].get<int>();
  _index = object[JSON_SUB_ADDRESS_NODE].get<int>();
  _thrown = object[JSON_STATE_NODE].get<bool>();
  _type = (TurnoutType)object[JSON_TYPE_NODE].get<int>();
  _boardAddress = 0;
  if (_index == -1)
  {
    // convert the provided decoder address to a board address and accessory index
    encodeDCCAccessoryAddress(&_boardAddress, &_index, _address);
    LOG(VERBOSE
      , "[Turnout %d] Loaded using DCC address %d as type %s and last known state of %s"
      , _turnoutID
      , _address
      , TURNOUT_TYPE_STRINGS[_type]
      , _thrown ? JSON_VALUE_THROWN : JSON_VALUE_CLOSED);
  } else {
    LOG(VERBOSE
      , "[Turnout %d] Loaded using address %d:%d as type %s and last known state of %s"
      , _turnoutID
      , _address
      , _index
      , TURNOUT_TYPE_STRINGS[_type]
      , _thrown ? JSON_VALUE_THROWN : JSON_VALUE_CLOSED);
  }
}

void Turnout::update(uint16_t address, int8_t index, TurnoutType type)
{
  _address = address;
  _index = index;
  _type = type;
  if (index == -1)
  {
    // convert the provided decoder address to a board address and accessory index
    encodeDCCAccessoryAddress(&_boardAddress, &_index, _address);
    LOG(VERBOSE
      , "[Turnout %d] Updated to use DCC address %d and type %s"
      , _turnoutID
      , _address
      , TURNOUT_TYPE_STRINGS[_type]);
  }
  else
  {
    LOG(VERBOSE
      , "[Turnout %d] Updated to address %d:%d and type %s"
      , _turnoutID
      , _address
      , _index
      , TURNOUT_TYPE_STRINGS[_type]);
  }
}

std::string Turnout::toJson(bool readableStrings)
{
  json object;
  object[JSON_ID_NODE] = _turnoutID;
  object[JSON_ADDRESS_NODE] = _address;
  object[JSON_BOARD_ADDRESS_NODE] = _boardAddress;
  if (_boardAddress)
  {
    object[JSON_SUB_ADDRESS_NODE] = -1;
  }
  else
  {
    object[JSON_SUB_ADDRESS_NODE] = _index;
  }
  if (readableStrings)
  {
    object[JSON_STATE_NODE] = _thrown ? JSON_VALUE_THROWN : JSON_VALUE_CLOSED;
  }
  else
  {
    object[JSON_STATE_NODE] = _thrown;
  }
  object[JSON_TYPE_NODE] = (int)_type;
  return object.dump();
}

void Turnout::set(bool thrown, bool sendDCCPacket)
{
  _thrown = thrown;
  if (sendDCCPacket)
  {
    packet_processor_add_refresh_source(this);
  }
  LOG(VERBOSE
    , "[Turnout %d] Set to %s"
    , _turnoutID
    , _thrown ? JSON_VALUE_THROWN : JSON_VALUE_CLOSED);
}

string Turnout::getStateAsDCCpp()
{
  return StringPrintf("<H %d %d %d %d>", _turnoutID, _address, _index, _thrown);
}

void Turnout::get_next_packet(unsigned code, dcc::Packet* packet)
{
  packet->add_dcc_basic_accessory(_address, _thrown);

  // remove ourselves as turnouts are single fire sources
  packet_processor_remove_refresh_source(this);
}