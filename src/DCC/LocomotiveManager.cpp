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

static constexpr const char * OLD_ROSTER_JSON_FILE = "roster.json";
static constexpr const char * ROSTER_JSON_FILE = "locoroster.json";
static constexpr const char * ROSTER_ENTRY_JSON_FILE = "roster-%d.json";

static constexpr const char * OLD_CONSISTS_JSON_FILE = "consists.json";
static constexpr const char * CONSISTS_JSON_FILE = "lococonsists.json";
static constexpr const char * CONSIST_ENTRY_JSON_FILE = "consist-%d.json";

std::unique_ptr<LocomotiveManager> locoManager;

void ThrottleCommandAdapter::process(const std::vector<std::string> arguments)
{
  locoManager->processThrottle(arguments);
}

void ThrottleExCommandAdapter::process(const std::vector<std::string> arguments)
{
  locoManager->processThrottleEx(arguments);
}

void FunctionCommandAdapter::process(const std::vector<std::string> arguments)
{
  locoManager->processFunction(arguments);
}

void FunctionExCommandAdapter::process(const std::vector<std::string> arguments)
{
  locoManager->processFunctionEx(arguments);
}


void LocomotiveManager::processThrottle(const vector<string> arguments)
{
  int registerNumber = std::stoi(arguments[0]);
  uint16_t locoAddress = std::stoi(arguments[1]);
  if(isConsistAddress(locoAddress) || isAddressInConsist(locoAddress))
  {
    processConsistThrottle(arguments);
    return;
  }
  Locomotive *instance = getLocomotiveByRegister(registerNumber);
  if(instance == nullptr)
  {
    instance = new Locomotive(locoAddress, trainService_);
    locos_.emplace_back(instance);
  }
  dcc::SpeedType speed;
  speed.set_dcc_128(std::stoi(arguments[2]));
  if (!std::stoi(arguments[3]))
  {
    speed.set_direction(dcc::SpeedType::REVERSE);
  }
  instance->set_speed(speed);
  instance->showStatus();
}

void LocomotiveManager::processThrottleEx(const vector<string> arguments)
{
  uint16_t locoAddress(std::stoi(arguments[0]));
  int8_t req_speed(std::stoi(arguments[1]));
  int8_t req_dir(std::stoi(arguments[2]));
  auto instance(getLocomotive(locoAddress));

  dcc::SpeedType upd_speed(instance->get_speed());
  if (req_speed >= 0)
  {
    upd_speed.set_dcc_128(req_speed);
  }
  if (req_dir >= 0)
  {
    upd_speed.set_direction(req_dir ? dcc::SpeedType::FORWARD : dcc::SpeedType::REVERSE);
  }
  instance->set_speed(upd_speed);
}

// This method decodes the incoming function packet(s) to update the stored
// functinon states. Loco update will be sent afterwards.
void LocomotiveManager::processFunction(const vector<string> arguments)
{
  uint16_t locoAddress = std::stoi(arguments[0]);
  uint8_t functionByte = std::stoi(arguments[1]);
  if(isConsistAddress(locoAddress))
  {
    return;
  }
  auto loco = getLocomotive(locoAddress);
  uint8_t firstFunction = 1, lastFunction = 4;
  uint8_t bits = functionByte;
  // check this is a request for functions F13-F28
  if(arguments.size() > 2)
  {
    bits = std::stoi(arguments[2]);
    if((functionByte & 0xDE) == 0xDE)
    {
      firstFunction = 13;
      lastFunction = 20;
    }
    else
    {
      firstFunction = 21;
      lastFunction = 28;
    }
  }
  else
  {
    // this is a request for functions FL,F1-F12
    // for safety this guarantees that first nibble of function byte will always
    // be of binary form 10XX which should always be the case for FL,F1-F12
    if((functionByte & 0xB0) == 0xB0)
    {
      firstFunction = 5;
      lastFunction = 8;
    }
    else if((functionByte & 0xA0) == 0xA0)
    {
      firstFunction = 9;
      lastFunction = 12;
    }
    else
    {
      loco->set_fn(0, bitRead(functionByte, 4));
    }
  }
  for(uint8_t funcID = firstFunction; funcID <= lastFunction; funcID++)
  {
    loco->set_fn(funcID, bitRead(bits, funcID - firstFunction));
  }
}

void LocomotiveManager::processFunctionEx(const vector<string> arguments)
{
  int locoAddress = std::stoi(arguments[0]);
  int function = std::stoi(arguments[1]);
  int state = std::stoi(arguments[2]);
  if(isConsistAddress(locoAddress))
  {
    return;
  }
  auto loco = getLocomotive(locoAddress);
  loco->set_fn(function, state);
}

void LocomotiveManager::processConsistThrottle(const vector<string> arguments)
{
  uint16_t locoAddress = std::stoi(arguments[1]);
  int8_t speed = std::stoi(arguments[2]);
  bool forward = arguments[3][0] == '1';
  for (const auto& consist : consists_)
  {
    if (consist->legacy_address() == locoAddress || consist->isAddressInConsist(locoAddress))
    {
      consist->updateThrottle(locoAddress, speed, forward);
      return;
    }
  }
}

void LocomotiveManager::showStatus()
{
  AtomicHolder h(this);
  for (const auto& loco : locos_)
  {
    loco->showStatus();
  }
  showConsistStatus();
}

void LocomotiveManager::showConsistStatus()
{
  AtomicHolder h(this);
  for (const auto& consist : consists_)
  {
    consist->showStatus();
  }
}

Locomotive *LocomotiveManager::getLocomotive(const uint16_t address, const bool managed)
{
  AtomicHolder h(this);
  auto const &elem = std::find_if(locos_.begin(), locos_.end(),
    [address](unique_ptr<Locomotive> & loco) -> bool
    {
      return (loco->legacy_address() == address);
    }
  );
  if (elem != locos_.end())
  {
    return elem->get();
  }
  locos_.emplace_back(new Locomotive(address, trainService_));
  return getLocomotive(address, managed);
}

Locomotive *LocomotiveManager::getLocomotiveByRegister(const uint8_t registerNumber)
{
  AtomicHolder h(this);
  auto const &elem = std::find_if(locos_.begin(), locos_.end(),
    [registerNumber](unique_ptr<Locomotive> & loco) -> bool
    {
      return (loco->getRegister() == registerNumber);
    }
  );
  if (elem != locos_.end())
  {
    return elem->get();
  }
  return nullptr;
}

void LocomotiveManager::removeLocomotive(const uint16_t address)
{
  AtomicHolder h(this);
  auto const &elem = std::find_if(locos_.begin(), locos_.end(),
    [address](unique_ptr<Locomotive> & loco) -> bool
    {
      return (loco->legacy_address() == address);
    }
  );
  if (elem != locos_.end())
  {
    locos_.erase(elem);
  }
}

bool LocomotiveManager::removeLocomotiveConsist(const uint16_t address)
{
  AtomicHolder h(this);
  auto const &elem = std::find_if(consists_.begin(), consists_.end(),
    [address](unique_ptr<LocomotiveConsist> & consist) -> bool
    {
      return (consist->legacy_address() == address);
    }
  );
  if (elem != consists_.end())
  {
    consists_.erase(elem);
    return true;
  }
  return false;
}

LocomotiveManager::LocomotiveManager(Node *node, TrainService *trainService) :
  BitEventInterface(Defs::CLEAR_EMERGENCY_STOP_EVENT
                  , Defs::EMERGENCY_STOP_EVENT)
, node_(node)
, trainService_(trainService)
{
  bool persistNeeded = false;
  LOG(INFO, "[Roster] Initializing Locomotive Roster");
  json root = json::parse(configStore->load(ROSTER_JSON_FILE));
  if (root.contains(JSON_COUNT_NODE))
  {
    uint16_t locoCount = root[JSON_COUNT_NODE].get<uint16_t>();
    infoScreen->replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "Found %02d Locos", locoCount);
    LOG(INFO, "[Roster] Loading %d Locomotive Roster entries", locoCount);
    for (auto entry : root[JSON_LOCOS_NODE])
    {
      std::string file = entry[JSON_FILE_NODE].get<string>();
      if (configStore->exists(file.c_str()))
      {
        string entry = configStore->load(file);
        roster_.emplace_back(new RosterEntry(entry));
      }
      else
      {
        LOG_ERROR("[Roster] Unable to locate Locomotive Roster entry %s!", file.c_str());
      }
    }
  }

  root = json::parse(configStore->load(OLD_ROSTER_JSON_FILE));
  if (root.contains(JSON_COUNT_NODE))
  {
    uint16_t locoCount = root[JSON_COUNT_NODE].get<uint16_t>();
    LOG(INFO, "[Roster] Loading %d older version Locomotive Roster entries", locoCount);
    infoScreen->replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "Load %02d Locos", locoCount);
    for (auto entry : root[JSON_LOCOS_NODE])
    {
      string data = entry.dump();
      roster_.emplace_back(new RosterEntry(data));
    }
    configStore->remove(OLD_ROSTER_JSON_FILE);
    persistNeeded = true;
  }
  LOG(INFO, "[Roster] Loaded %d Locomotive Roster entries", roster_.size());

  root = json::parse(configStore->load(CONSISTS_JSON_FILE));
  if (root.contains(JSON_COUNT_NODE))
  {
    uint16_t consistCount = root[JSON_COUNT_NODE].get<uint16_t>();
    LOG(INFO, "[Consist] Loading %d Locomotive Consists", consistCount);
    infoScreen->replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "Load %02d Consists", consistCount);
    for(auto entry : root[JSON_CONSISTS_NODE])
    {
      string file = entry[JSON_FILE_NODE].get<string>();
      if (configStore->exists(file.c_str()))
      {
        string entry = configStore->load(file);
        consists_.emplace_back(LocomotiveConsist::fromJson(entry, trainService_));
      }
      else
      {
        LOG_ERROR("[Consist] Unable to locate Locomotive Consist Entry %s!", file.c_str());
      }
    }
  }

  root = json::parse(configStore->load(OLD_CONSISTS_JSON_FILE));
  if (root.contains(JSON_COUNT_NODE))
  {
    uint16_t consistCount = root[JSON_COUNT_NODE].get<uint16_t>();
    LOG(INFO, "[Consist] Loading %d Locomotive Consists", consistCount);
    infoScreen->replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "Load %02d Consists", consistCount);
    for (auto entry : root[JSON_CONSISTS_NODE])
    {
      string data = entry.dump();
      consists_.emplace_back(LocomotiveConsist::fromJson(data, trainService_));
    }
    configStore->remove(OLD_CONSISTS_JSON_FILE);
    persistNeeded = true;
  }

  LOG(INFO, "[Consist] Loaded %d Locomotive Consists", consists_.size());
  if (persistNeeded)
  {
    store();
  }
}

void LocomotiveManager::clear() {
  AtomicHolder h(this);
  for (auto & e : locos_)
  {
    e.reset(nullptr);
  }
  for (auto & e : consists_)
  {
    e.reset(nullptr);
  }
  for (auto & e : roster_)
  {
    e.reset(nullptr);
  }
  locos_.clear();
  consists_.clear();
  roster_.clear();
}

uint16_t LocomotiveManager::store()
{
  AtomicHolder h(this);
  json locoRoot;
  uint16_t locoStoredCount = 0;
  for (const auto& entry : roster_)
  {
    string filename = StringPrintf(ROSTER_ENTRY_JSON_FILE, entry->getAddress());
    locoRoot[JSON_LOCOS_NODE].push_back(filename.c_str());
    json content = entry->toJson();
    configStore->store(filename.c_str(), content);
    locoStoredCount++;
  }
  locoRoot[JSON_COUNT_NODE] = locoStoredCount;
  configStore->store(ROSTER_JSON_FILE, locoRoot);

  json consistRoot;
  uint16_t consistStoredCount = 0;
  for (const auto& consist : consists_)
  {
    string filename = StringPrintf(CONSIST_ENTRY_JSON_FILE, consist->legacy_address());
    consistRoot[JSON_FILE_NODE].push_back(filename.c_str());
    json content = consist->toJson();
    configStore->store(filename.c_str(), content);
    consistStoredCount++;
  }
  consistRoot[JSON_COUNT_NODE] = consistStoredCount;
  configStore->store(CONSISTS_JSON_FILE, consistRoot);
  return locoStoredCount + consistStoredCount;
}

vector<RosterEntry *> LocomotiveManager::getDefaultLocos(const int8_t maxCount)
{
  AtomicHolder h(this);
  vector<RosterEntry *> retval;
  for (const auto& entry : roster_)
  {
    if(entry->isDefaultOnThrottles())
    {
      if(maxCount < 0 || (maxCount > 0 && retval.size() < maxCount))
      {
        retval.push_back(entry.get());
      }
    }
  }
  return retval;
}

std::string LocomotiveManager::getDefaultLocosAsJson()
{
  AtomicHolder h(this);
  json root;
  for (const auto& entry : roster_)
  {
    if(entry->isDefaultOnThrottles())
    {
      root.push_back(entry->toJson());
    }
  }
  return root.dump();
}

std::string LocomotiveManager::getActiveLocosAsJson()
{
  AtomicHolder h(this);
  json root;
  for (const auto& loco : locos_)
  {
    root.push_back(loco->toJson(false));
  }
  for (const auto& consist : consists_)
  {
    root.push_back(consist->toJson(false));
  }
  return root.dump();
}

std::string LocomotiveManager::getRosterEntriesAsJson()
{
  AtomicHolder h(this);
  json root;
  for (const auto& entry : roster_) {
    root.push_back(entry->toJson());
  }
  return root.dump();
}

bool LocomotiveManager::isConsistAddress(uint16_t address)
{
  AtomicHolder h(this);
  for (const auto& consist : consists_)
  {
    if(consist->legacy_address() == address)
    {
      return true;
    }
  }
  return false;
}

bool LocomotiveManager::isAddressInConsist(uint16_t address)
{
  AtomicHolder h(this);
  for (const auto& consist : consists_)
  {
    if(consist->isAddressInConsist(address))
    {
      return true;
    }
  }
  return false;
}

LocomotiveConsist *LocomotiveManager::getConsistByID(uint8_t consistAddress)
{
  AtomicHolder h(this);
  for (const auto& consist : consists_)
  {
    if(consist->legacy_address() == consistAddress)
    {
      return consist.get();
    }
  }
  return nullptr;
}

LocomotiveConsist *LocomotiveManager::getConsistForLoco(uint16_t locomotiveAddress)
{
  AtomicHolder h(this);
  for (const auto& consist : consists_)
  {
    if(consist->isAddressInConsist(locomotiveAddress))
    {
      return consist.get();
    }
  }
  return nullptr;
}

LocomotiveConsist *LocomotiveManager::createLocomotiveConsist(int8_t consistAddress)
{
  AtomicHolder h(this);
  if(consistAddress == 0)
  {
    LOG(INFO,
        "[Consist] Creating new Loco Consist, automatic address selection...");
    uint8_t newConsistAddress = 127;
    for (const auto& consist : consists_)
    {
      if(newConsistAddress > consist->legacy_address() - 1 &&
         !isConsistAddress(consist->legacy_address() - 1))
      {
        newConsistAddress = consist->legacy_address() - 1;
        LOG(INFO, "[Consist] Found free address for new Loco Consist: %d"
          , newConsistAddress);
        break;
      }
    }
    if(newConsistAddress > 0)
    {
      LOG(INFO, "[Consist] Adding new Loco Consist %d", newConsistAddress);
      consists_.emplace_back(new LocomotiveConsist(newConsistAddress, trainService_, true));
      return getConsistByID(newConsistAddress);
    }
    else
    {
      LOG(INFO,
          "[Consist] Unable to locate free address for new Loco Consist, "
          "giving up.");
    }
  }
  else
  {
    LOG(INFO, "[Consist] Adding new Loco Consist %d", consistAddress);
    consists_.emplace_back(new LocomotiveConsist(abs(consistAddress)
                                               , trainService_
                                               , consistAddress < 0));
    return getConsistByID(abs(consistAddress));
  }
  return nullptr;
}

RosterEntry *LocomotiveManager::getRosterEntry(uint16_t address, bool create)
{
  AtomicHolder h(this);
  auto const &elem = std::find_if(roster_.begin(), roster_.end(),
    [address](unique_ptr<RosterEntry> & entry) -> bool
    {
      return (entry->getAddress() == address);
    }
  );
  if (elem != roster_.end())
  {
    return elem->get();
  }
  if (!create)
  {
    return nullptr;
  }
  LOG(VERBOSE, "[Roster] No roster entry for address %d, creating", address);
  roster_.emplace_back(new RosterEntry(address));
  return getRosterEntry(address, create);
}

void LocomotiveManager::removeRosterEntry(uint16_t address)
{
  AtomicHolder h(this);
  auto const &elem = std::find_if(roster_.begin(), roster_.end(),
    [address](unique_ptr<RosterEntry> & entry) -> bool
    {
      return (entry->getAddress() == address);
    }
  );
  if (elem != roster_.end())
  {
    LOG(VERBOSE, "[Roster] Removing roster entry for address %d", address);
    elem->reset(nullptr);
    roster_.erase(elem);
    return;
  }
  LOG(WARNING
    , "[Roster] Roster entry for address %d doesn't exist, ignoring "
      "delete request"
    , address);
}

RosterEntry::RosterEntry(string &content)
{
  json object = json::parse(content);
  _description = object[JSON_DESCRIPTION_NODE].get<string>();
  _address = object[JSON_ADDRESS_NODE];
  _type = object[JSON_TYPE_NODE].get<string>();
  _idleOnStartup = object[JSON_IDLE_ON_STARTUP_NODE] == JSON_VALUE_TRUE;
  _defaultOnThrottles = object[JSON_DEFAULT_ON_THROTTLE_NODE] == JSON_VALUE_TRUE;
}

std::string RosterEntry::toJson()
{
  json object;
  object[JSON_DESCRIPTION_NODE] = _description;
  object[JSON_ADDRESS_NODE] = _address;
  object[JSON_TYPE_NODE] = _type;
  object[JSON_IDLE_ON_STARTUP_NODE] = _idleOnStartup ? JSON_VALUE_TRUE : JSON_VALUE_FALSE;
  object[JSON_DEFAULT_ON_THROTTLE_NODE] = _defaultOnThrottles ? JSON_VALUE_TRUE : JSON_VALUE_FALSE;
  return object.dump();
}

RosterEntry::~RosterEntry()
{
  string filename = StringPrintf(ROSTER_ENTRY_JSON_FILE, getAddress());
  if(configStore->exists(filename.c_str()))
  {
    configStore->remove(filename.c_str());
  }
}