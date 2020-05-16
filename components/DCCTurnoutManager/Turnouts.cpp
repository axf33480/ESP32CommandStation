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

#include "Turnouts.h"

#include <ConfigurationManager.h>
#include <dcc/DccDebug.hxx>
#include <dcc/UpdateLoop.hxx>
#include <JsonConstants.h>
#include <json.hpp>

using nlohmann::json;

std::unique_ptr<TurnoutManager> turnoutManager;

static constexpr const char * TURNOUTS_JSON_FILE = "turnouts.json";

static constexpr const char *TURNOUT_TYPE_STRINGS[] =
{
  "LEFT",
  "RIGHT",
  "WYE",
  "MULTI"
};

TurnoutManager::TurnoutManager(openlcb::Node *node, Service *service)
  : turnoutEventConsumer_(node, this)
  , persistFlow_(service, SEC_TO_NSEC(CONFIG_TURNOUT_PERSISTENCE_INTERVAL_SEC)
              , std::bind(&TurnoutManager::persist, this))
  , dirty_(false)
{
  OSMutexLock h(&mux_);
  LOG(INFO, "[Turnout] Initializing DCC Turnout database");
  json root = json::parse(
    Singleton<ConfigurationManager>::instance()->load(TURNOUTS_JSON_FILE));
  for (auto turnout : root)
  {
    turnouts_.push_back(
      std::make_unique<Turnout>(turnout[JSON_ADDRESS_NODE].get<int>()
                              , turnout[JSON_STATE_NODE].get<int>()
                              , (TurnoutType)turnout[JSON_TYPE_NODE].get<int>()));
  }
  LOG(INFO, "[Turnout] Loaded %d DCC turnout(s)", turnouts_.size());
}

void TurnoutManager::clear()
{
  OSMutexLock h(&mux_);
  for (auto & turnout : turnouts_)
  {
    turnout.reset(nullptr);
  }
  turnouts_.clear();
  dirty_ = true;
}

string TurnoutManager::set(uint16_t address, bool thrown, bool sendDCC)
{
  OSMutexLock h(&mux_);
  for (auto & turnout : turnouts_)
  {
    if (turnout->getAddress() == address)
    {
      turnout->set(thrown, sendDCC);
      return turnout->get_state_for_dccpp();
    }
  }

  // we didn't find it, create it and set it
  turnouts_.push_back(std::make_unique<Turnout>(turnouts_.size() + 1, address));
  dirty_ = true;
  return set(address, thrown, sendDCC);
}

string TurnoutManager::toggle(uint16_t address)
{
  OSMutexLock h(&mux_);
  auto const &elem = std::find_if(turnouts_.begin(), turnouts_.end(),
    [address](std::unique_ptr<Turnout> & turnout) -> bool
    {
      return (turnout->getAddress() == address);
    }
  );
  if (elem != turnouts_.end())
  {
    elem->get()->toggle();
    return elem->get()->get_state_for_dccpp();
  }

  // we didn't find it, create it and throw it
  turnouts_.push_back(std::make_unique<Turnout>(address, -1));
  dirty_ = true;
  return toggle(address);
}

string TurnoutManager::getStateAsJson(bool readable)
{
  OSMutexLock h(&mux_);
  return get_state_as_json(readable);
}

string TurnoutManager::get_state_for_dccpp()
{
  OSMutexLock h(&mux_);
  if (turnouts_.empty())
  {
    return COMMAND_FAILED_RESPONSE;
  }
  string status;
  for (auto& turnout : turnouts_)
  {
    status += turnout->get_state_for_dccpp(true);
  }
  return status;
}

Turnout *TurnoutManager::createOrUpdate(const uint16_t address
                                      , const TurnoutType type)
{
  OSMutexLock h(&mux_);
  auto const &elem = std::find_if(turnouts_.begin(), turnouts_.end(),
    [address](std::unique_ptr<Turnout> & turnout) -> bool
    {
      return (turnout->getAddress() == address);
    }
  );
  if (elem != turnouts_.end())
  {
    elem->get()->update(address, type);
    return elem->get();
  }
  // we didn't find it, create it!
  turnouts_.push_back(std::make_unique<Turnout>(address, false, type));
  dirty_ = true;
  return turnouts_.back().get();
}

bool TurnoutManager::remove(const uint16_t address)
{
  OSMutexLock h(&mux_);
  auto const &elem = std::find_if(turnouts_.begin(), turnouts_.end(),
    [address](std::unique_ptr<Turnout> & turnout) -> bool
    {
      return (turnout->getAddress() == address);
    }
  );
  if (elem != turnouts_.end())
  {
    LOG(CONFIG_TURNOUT_LOG_LEVEL
      , "[Turnout %d] Deleted", address);
    turnouts_.erase(elem);
    dirty_ = true;
    return true;
  }
  return false;
}

Turnout *TurnoutManager::getByIndex(const uint16_t index)
{
  OSMutexLock h(&mux_);
  if (index < turnouts_.size())
  {
    return turnouts_[index].get();
  }
  return nullptr;
}

Turnout *TurnoutManager::get(const uint16_t address)
{
  OSMutexLock h(&mux_);
  auto const &elem = std::find_if(turnouts_.begin(), turnouts_.end(),
    [address](std::unique_ptr<Turnout> & turnout) -> bool
    {
      return (turnout->getAddress() == address);
    }
  );
  if (elem != turnouts_.end())
  {
    return elem->get();
  }
  return nullptr;
}

uint16_t TurnoutManager::count()
{
  OSMutexLock h(&mux_);
  return turnouts_.size();
}

// TODO: shift this to consume the LCC event directly
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
    // packet data format:
    // payload[0]  payload[1]
    // 10aaaaaa    1AAACDDD
    // ^ ^^^^^^    ^^^^^^^^ 
    // | |         ||  || |
    // | |         ||  || \-state bit
    // | |         ||  |\-output index
    // | |         ||  \-activate/deactivate output flag (ignored)
    // | |         |\-board address most significant three bits
    // | |         |  stored in 1s complement (1=0, 0=1)
    // | |         \-accessory packet flag
    // | \-board address (least significant six bits)
    // \-accessory packet flag
    // converting back to a single address using the following: AAAaaaaaaDDD
    // note that only the output index is used in calculation of the final
    // address since only the base address is stored in the CS.
    uint16_t boardAddress = ((~pkt->payload[1] & 0b01110000) << 2) |
                            (pkt->payload[0] & 0b00111111);
    uint8_t boardIndex = (pkt->payload[1] & 0b00000110) >> 1;
    // least significant bit of the second byte is thrown/closed indicator.
    bool state = pkt->payload[1] & 0b00000001;
    // Set the turnout to the requested state, don't send a DCC packet.
    set(decodeDCCAccessoryAddress(boardAddress, boardIndex), state);
  }
  b->unref();
}

string TurnoutManager::get_state_as_json(bool readableStrings)
{
  string content = "[";
  for (const auto& turnout : turnouts_)
  {
    // only add the seperator if we have already serialized at least one
    // turnout.
    if (content.length() > 1)
    {
      content += ",";
    }
    content += turnout->toJson(readableStrings);
  }
  content += "]";
  return content;
}

void TurnoutManager::persist()
{
  // Check if we have any changes to persist, if not exit early to reduce
  // unnecessary wear on the flash when running on SPIFFS.
  OSMutexLock h(&mux_);
  if (!dirty_)
  {
    return;
  }
  Singleton<ConfigurationManager>::instance()->store(TURNOUTS_JSON_FILE, get_state_as_json(false));
  dirty_ = false;
}

void encodeDCCAccessoryAddress(uint16_t *boardAddress, int8_t *boardIndex
                             , uint16_t address)
{
  // DCC address starts at 1, board address is 0-511 and index is 0-3.
  *boardAddress = ((address - 1) / 4) + 1;
  *boardIndex = (address - 1) % 4;
}

uint16_t decodeDCCAccessoryAddress(uint16_t boardAddress, int8_t boardIndex)
{
  // when board address is zero we need to only use the index.
  if (boardAddress == 0)
  {
    return boardIndex + 1;
  }
  // convert the address:index to a single address for the decoder.
  return ((boardAddress << 2) | boardIndex) + 1;
}

Turnout::Turnout(uint16_t address, bool thrown, TurnoutType type)
               : _address(address), _thrown(thrown), _type(type)
{
  LOG(INFO, "[Turnout %d] Registered as type %s and initial state of %s"
    , _address, TURNOUT_TYPE_STRINGS[_type]
    , _thrown ? JSON_VALUE_THROWN : JSON_VALUE_CLOSED);
}

void Turnout::update(uint16_t address, TurnoutType type)
{
  _address = address;
  _type = type;
  LOG(CONFIG_TURNOUT_LOG_LEVEL, "[Turnout %d] Updated type %s", _address
    , TURNOUT_TYPE_STRINGS[_type]);
}

string Turnout::toJson(bool readableStrings)
{
  string serialized = StringPrintf("{\"%s\":%d,\"%s\":%d,\"%s\":"
  , JSON_ADDRESS_NODE, _address, JSON_TYPE_NODE, _type, JSON_STATE_NODE);
  if (readableStrings)
  {
    serialized += StringPrintf("\"%s\"", _thrown ? JSON_VALUE_THROWN
                                                 : JSON_VALUE_CLOSED);
  }
  else
  {
    serialized += integer_to_string(_thrown);
  }
  serialized += "}";
  return serialized;
}

void Turnout::set(bool thrown, bool sendDCCPacket)
{
  _thrown = thrown;
  if (sendDCCPacket)
  {
    packet_processor_add_refresh_source(this);
  }
  LOG(CONFIG_TURNOUT_LOG_LEVEL, "[Turnout %d] Set to %s", _address
    , _thrown ? JSON_VALUE_THROWN : JSON_VALUE_CLOSED);
}

string Turnout::get_state_for_dccpp(bool include_board_index)
{
  if (include_board_index)
  {
    int8_t index;
    uint16_t board;
    encodeDCCAccessoryAddress(&board, &index, _address);
    return StringPrintf("<H %d %d %d %d>", _address, board, index, _thrown);
  }
  return StringPrintf("<H %d %d>", _address, _thrown);
}

void Turnout::get_next_packet(unsigned code, dcc::Packet* packet)
{
  // shift the address to make room for the thrown flag.
  packet->add_dcc_basic_accessory((_address << 1) + _thrown, true);

#ifdef CONFIG_TURNOUT_LOGGING_VERBOSE
  LOG(INFO, "[Turnout %d] Packet: %s", _address
    , packet_to_string(*packet, true).c_str());
#endif

  // remove ourselves as turnouts are single fire sources
  packet_processor_remove_refresh_source(this);
}