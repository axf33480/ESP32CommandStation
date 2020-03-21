/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2019-2020 Mike Dunston

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
#include "ESP32TrainDatabase.h"

#include <CDIHelper.h>

#include <json.hpp>
#include <utils/FileUtils.hxx>

#include "TrainDbCdi.hxx"

namespace commandstation
{
// JSON serialization mappings for the commandstation::DccMode enum
NLOHMANN_JSON_SERIALIZE_ENUM(DccMode,
{
  { DCCMODE_DEFAULT,     "DCC" },
  { DCCMODE_OLCBUSER,    "DCC-OlcbUser" },
  { MARKLIN_DEFAULT,     "Marklin" },
  { MARKLIN_OLD,         "Marklin (v1)" },
  { MARKLIN_NEW,         "Marklin (v2, f0-f4)" },
  { MARKLIN_TWOADDR,     "Marklin (v2, f0-f8)" },
  { MFX,                 "Marklin (MFX)" },
  { DCC_DEFAULT,         "DCC"},
  { DCC_14,              "DCC (14 speed step)"},
  { DCC_28,              "DCC (28 speed step)"},
  { DCC_128,             "DCC (128 speed step)"},
  { DCC_14_LONG_ADDRESS, "DCC (14 speed step, long address)"},
  { DCC_28_LONG_ADDRESS, "DCC (28 speed step, long address)"},
  { DCC_128_LONG_ADDRESS,"DCC (128 speed step, long address)"},
});

// JSON serialization mappings for the commandstation::Symbols enum
NLOHMANN_JSON_SERIALIZE_ENUM(Symbols,
{
  { FN_NONEXISTANT,   "N/A" },
  { LIGHT,            "Light" },
  { BEAMER,           "Beamer" },
  { BELL,             "Bell" },
  { HORN,             "Horn" },
  { SHUNT,            "Shunting mode" },
  { PANTO,            "Pantograph" },
  { SMOKE,            "Smoke" },
  { ABV,              "Momentum On/Off" },
  { WHISTLE,          "Whistle" },
  { SOUND,            "Sound" },
  { FNT11,            "Generic Function" },
  { SPEECH,           "Announce" },
  { ENGINE,           "Engine" },
  { LIGHT1,           "Light1" },
  { LIGHT2,           "Light2" },
  { TELEX,            "Coupler" },
  { FN_UNKNOWN,       "Unknown" },
  { MOMENTARY,        "momentary" },
  { FNP,              "fnp" },
  { SOUNDP,           "soundp" },
  { FN_UNINITIALIZED, "uninit" },
})
}

namespace esp32cs
{

using nlohmann::json;

static constexpr char TRAIN_CDI_FILE[] = "/cfg/LCC/train.xml";
static constexpr char TEMP_TRAIN_CDI_FILE[] = "/cfg/LCC/tmptrain.xml";

// This should really be defined inside TractionDefs.hxx and used by the call
// to TractionDefs::train_node_id_from_legacy().
static constexpr uint64_t const OLCB_NODE_ID_USER = 0x050101010000ULL;

// converts a Esp32PersistentTrainData to a json object
void to_json(json& j, const Esp32PersistentTrainData& t)
{
  j = json({
    { JSON_NAME_NODE, t.name },
    { JSON_ADDRESS_NODE, t.address },
    { JSON_IDLE_ON_STARTUP_NODE, t.automatic_idle },
    { JSON_DEFAULT_ON_THROTTLE_NODE, t.show_on_limited_throttles },
    { JSON_FUNCTIONS_NODE, t.functions },
    { JSON_MODE_NODE, t.mode },
  });
}

// converts a json payload to a Esp32PersistentTrainData object
void from_json(const json& j, Esp32PersistentTrainData& t)
{
  j.at(JSON_NAME_NODE).get_to(t.name);
  j.at(JSON_ADDRESS_NODE).get_to(t.address);
  j.at(JSON_IDLE_ON_STARTUP_NODE).get_to(t.automatic_idle);
  j.at(JSON_DEFAULT_ON_THROTTLE_NODE).get_to(t.show_on_limited_throttles);
  j.at(JSON_FUNCTIONS_NODE).get_to(t.functions);
  j.at(JSON_MODE_NODE).get_to(t.mode);
}

Esp32TrainDbEntry::Esp32TrainDbEntry(Esp32PersistentTrainData data
                                   , bool persist)
  : data_(data), dirty_(true), persist_(persist)
{
  recalcuate_max_fn();
  LOG(INFO, "[Loco:%s] Locomotive '%s' created", identifier().c_str()
    , data_.name.c_str());
}

string Esp32TrainDbEntry::identifier()
{
  dcc::TrainAddressType addrType =
    dcc_mode_to_address_type(data_.mode, data_.address);
  if (addrType == dcc::TrainAddressType::DCC_SHORT_ADDRESS ||
      addrType == dcc::TrainAddressType::DCC_LONG_ADDRESS)
  {
    string prefix = "long_address";
    if (addrType == dcc::TrainAddressType::DCC_SHORT_ADDRESS)
    {
      prefix = "short_address";
    }
    if ((data_.mode & DCC_SS_MASK) == 1)
    {
      return StringPrintf("dcc_14/%s/%d", prefix.c_str(), data_.address);
    }
    else if ((data_.mode & DCC_SS_MASK) == 2)
    {
      return StringPrintf("dcc_28/%s/%d", prefix.c_str(), data_.address);
    }
    else
    {
      return StringPrintf("dcc_128/%s/%d", prefix.c_str(), data_.address);
    }
  }
  else if (addrType == dcc::TrainAddressType::MM)
  {
    // TBD: should marklin types be explored further?
    return StringPrintf("marklin/%d", data_.address);
  }
  return StringPrintf("unknown/%d", data_.address);
}

openlcb::NodeID Esp32TrainDbEntry::get_traction_node()
{
  if (data_.mode == DCCMODE_OLCBUSER)
  {
    return OLCB_NODE_ID_USER | static_cast<openlcb::NodeID>(data_.address);
  }
  else
  {
    return openlcb::TractionDefs::train_node_id_from_legacy(
        dcc_mode_to_address_type(data_.mode, data_.address), data_.address);
  }
}

unsigned Esp32TrainDbEntry::get_function_label(unsigned fn_id)
{
  // if the function id is larger than our max list reject it
  if (fn_id > maxFn_)
  {
    return FN_NONEXISTANT;
  }
  // return the mapping for the function
  return data_.functions[fn_id];
}

void Esp32TrainDbEntry::recalcuate_max_fn()
{
  // recalculate the maxFn_ based on the first occurrence of FN_NONEXISTANT
  // and if not found default to the size of the functions labels vector.
  auto e = std::find_if(data_.functions.begin(), data_.functions.end()
                      , [](const Symbols &e)
  {
    return e == FN_NONEXISTANT;
  });

  if (e != data_.functions.end())
  {
    maxFn_ = std::distance(data_.functions.begin(), e);
  }
  else
  {
    maxFn_ = data_.functions.size();
  }
}

static constexpr const char * TRAIN_DB_JSON_FILE = "trains.json";
static constexpr const char * LEGACY_ROSTER_JSON_FILE = "roster.json";

Esp32TrainDatabase::Esp32TrainDatabase(openlcb::SimpleStackBase *stack)
{
  TrainConfigDef trainCfg(0);
  TrainTmpConfigDef tmpTrainCfg(0);
  CDIHelper::create_config_descriptor_xml(trainCfg, TRAIN_CDI_FILE);
  CDIHelper::create_config_descriptor_xml(tmpTrainCfg, TEMP_TRAIN_CDI_FILE);
  trainCdiFile_.reset(new openlcb::ROFileMemorySpace(TRAIN_CDI_FILE));
  tempTrainCdiFile_.reset(new openlcb::ROFileMemorySpace(TEMP_TRAIN_CDI_FILE));
  persistFlow_.emplace(stack->service()
                     , SEC_TO_NSEC(CONFIG_ROSTER_PERSISTENCE_INTERVAL_SEC)
                     , std::bind(&Esp32TrainDatabase::persist, this));

  LOG(INFO, "[TrainDB] Initializing...");
  if (Singleton<ConfigurationManager>::instance()->exists(TRAIN_DB_JSON_FILE))
  {
    auto roster =
      Singleton<ConfigurationManager>::instance()->load(TRAIN_DB_JSON_FILE);
    json stored_trains = json::parse(roster);
    for (auto &entry : stored_trains)
    {
      auto data = entry.get<Esp32PersistentTrainData>();
      auto ent = std::find_if(knownTrains_.begin(), knownTrains_.end(),
      [data](const shared_ptr<Esp32TrainDbEntry> &train)
      {
        return train->get_legacy_address() == data.address;
      });
      if (ent == knownTrains_.end())
      {
        LOG(INFO, "[TrainDB] Registering %u - %s (idle: %s, limited: %s)"
          , data.address, data.name.c_str()
          , data.automatic_idle ? JSON_VALUE_ON : JSON_VALUE_OFF
          , data.show_on_limited_throttles ? JSON_VALUE_ON : JSON_VALUE_OFF);
        auto train = new Esp32TrainDbEntry(data);
        train->reset_dirty();
        knownTrains_.emplace_back(train);
      }
      else
      {
        LOG_ERROR("[TrainDB] Duplicate roster entry detected for loco addr %u."
                , data.address);
      }
    }
  }

  if (Singleton<ConfigurationManager>::instance()->exists(LEGACY_ROSTER_JSON_FILE))
  {
    LOG(INFO, "[TrainDB] Loading Legacy roster file...");
    auto legacy_roster =
      Singleton<ConfigurationManager>::instance()->load(LEGACY_ROSTER_JSON_FILE);
    json roster = json::parse(legacy_roster);
    for (auto &entry : roster)
    {
      Esp32PersistentTrainData data;
      entry.at(JSON_ADDRESS_NODE).get_to(data.address);
      entry.at(JSON_DESCRIPTION_NODE).get_to(data.name);
      string auto_idle = entry[JSON_IDLE_ON_STARTUP_NODE];
      string limited_throttle = entry[JSON_DEFAULT_ON_THROTTLE_NODE];
      data.automatic_idle = (auto_idle == JSON_VALUE_TRUE);
      data.show_on_limited_throttles = (limited_throttle == JSON_VALUE_TRUE);
      auto ent = std::find_if(knownTrains_.begin(), knownTrains_.end(),
      [data](const shared_ptr<Esp32TrainDbEntry> &train)
      {
        LOG(INFO, "[TrainDB] Registering %u - %s (idle: %s, limited: %s)"
          , data.address, data.name.c_str()
          , data.automatic_idle ? JSON_VALUE_ON : JSON_VALUE_OFF
          , data.show_on_limited_throttles ? JSON_VALUE_ON : JSON_VALUE_OFF);
        return train->get_legacy_address() == data.address;
      });
      if (ent == knownTrains_.end())
      {
        knownTrains_.emplace_back(new Esp32TrainDbEntry(data));
      }
      else
      {
        LOG_ERROR("[TrainDB] Duplicate roster entry detected for loco addr %u."
                , data.address);
      }
    }
    legacyEntriesFound_ = true;
  }
  LOG(INFO, "[TrainDB] There are %d entries in the database."
    , knownTrains_.size());

  // add callback to register any existing roster entries after startup is
  // complete
  stack->executor()->add(new CallbackExecutable([]()
  {
    Singleton<Esp32TrainDatabase>::instance()->load_idle_trains();
  }));
}

void Esp32TrainDatabase::load_idle_trains()
{
  // TBD
}

std::shared_ptr<TrainDbEntry> Esp32TrainDatabase::create_if_not_found(unsigned address
                                                                    , DccMode mode)
{
  OSMutexLock l(&knownTrainsLock_);
  LOG(VERBOSE, "[TrainDB] Searching for roster entry for address: %u", address);
  auto entry = std::find_if(knownTrains_.begin(), knownTrains_.end(),
    [address](const shared_ptr<Esp32TrainDbEntry> &train)
    {
      return train->get_legacy_address() == (uint16_t)address;
    });
  if (entry != knownTrains_.end())
  {
    LOG(VERBOSE, "[TrainDB] Found existing entry:%s."
      , (*entry)->identifier().c_str());
    return *entry;
  }
  auto index = knownTrains_.size();
  knownTrains_.emplace_back(
    new Esp32TrainDbEntry(Esp32PersistentTrainData(address, mode)));
  LOG(VERBOSE, "[TrainDB] No entry was found, created new entry:%s."
    , knownTrains_[index]->identifier().c_str());
  return knownTrains_[index];
}

void Esp32TrainDatabase::delete_entry(unsigned address)
{
  OSMutexLock l(&knownTrainsLock_);
}

shared_ptr<TrainDbEntry> Esp32TrainDatabase::get_entry(unsigned train_id)
{
  OSMutexLock l(&knownTrainsLock_);
  LOG(VERBOSE, "[TrainDB] get_entry(%u) : %zu", train_id, knownTrains_.size());
  if (train_id < knownTrains_.size())
  {
    return knownTrains_[train_id];
  }
  return nullptr;
}

shared_ptr<TrainDbEntry> Esp32TrainDatabase::find_entry(openlcb::NodeID node_id
                                                      , unsigned hint)
{
  OSMutexLock l(&knownTrainsLock_);
  LOG(VERBOSE, "[TrainDB] Searching for Train Node:%s, Hint:%u"
    , uint64_to_string(node_id).c_str(), hint);
  auto entry = std::find_if(knownTrains_.begin(), knownTrains_.end(),
    [node_id, hint](const shared_ptr<Esp32TrainDbEntry> &train)
    {
      return train->get_traction_node() == node_id
          || train->get_legacy_address() == hint;
    });
  if (entry != knownTrains_.end())
  {
    LOG(VERBOSE, "[TrainDB] Found existing entry: %s."
      , (*entry)->identifier().c_str());
    return *entry;
  }
  LOG(VERBOSE, "[TrainDB] No entry found!");
  return nullptr;
}

// The caller of this method expects a zero based index into the vector. This
// may be changed in the future to use the loco address instead.
unsigned Esp32TrainDatabase::add_dynamic_entry(TrainDbEntry* temp_entry)
{
  uint16_t address = temp_entry->get_legacy_address();
  DccMode mode = temp_entry->get_legacy_drive_mode();
  OSMutexLock l(&knownTrainsLock_);
  delete temp_entry;

  // prevent duplicate entries in the roster
  auto ent = std::find_if(knownTrains_.begin(), knownTrains_.end(),
    [address](const shared_ptr<Esp32TrainDbEntry> &train)
    {
      return train->get_legacy_address() == address;
    });
  if (ent != knownTrains_.end())
  {
    return std::distance(knownTrains_.begin(), ent);
  }
  LOG(INFO, "[TrainDB] Creating roster entry for locomotive %u (mode:%d)."
    , address, mode);

  // track the index for the new train entry
  size_t index = knownTrains_.size();
  // create the new entry, by default the entry will be marked as dirty so that
  // it will be persisted as part of the next persistence check. If the
  // auto-create feature is not enabled the entry will not be persisted and is
  // not marked dirty.
  knownTrains_.emplace_back(
    new Esp32TrainDbEntry(Esp32PersistentTrainData(address, mode)
#if !CONFIG_ROSTER_AUTO_CREATE_ENTRIES
                        , false
#endif
  ));
  return index;
}

set<uint16_t> Esp32TrainDatabase::get_default_train_addresses(uint16_t limit)
{
  set<uint16_t> results;
  OSMutexLock l(&knownTrainsLock_);
  for(auto entry : knownTrains_)
  {
    if (entry->get_data().show_on_limited_throttles)
    {
      if (limit)
      {
        results.insert(entry->get_legacy_address());
        limit--;
      }
      else
      {
        break;
      }
    }
  }
  return results;
}

void Esp32TrainDatabase::set_train_name(unsigned address, std::string &name)
{
  OSMutexLock l(&knownTrainsLock_);
  LOG(VERBOSE, "[TrainDB] Searching for train with address %u", address);
  auto entry = std::find_if(knownTrains_.begin(), knownTrains_.end(),
    [address](const shared_ptr<Esp32TrainDbEntry> &train)
    {
      return train->get_legacy_address() == (uint16_t)address;
    });
  if (entry != knownTrains_.end())
  {
    LOG(VERBOSE, "[TrainDB] Setting train(%u) name: %s", address, name.c_str());
    (*entry)->set_train_name(name);
  }
  else
  {
    LOG_ERROR("[TrainDB] train %u not found, unable to set the name!", address);
  }
}

void Esp32TrainDatabase::set_train_auto_idle(unsigned address, bool idle)
{
  OSMutexLock l(&knownTrainsLock_);
  LOG(VERBOSE, "[TrainDB] Searching for train with address %u", address);
  auto entry = std::find_if(knownTrains_.begin(), knownTrains_.end(),
    [address](const shared_ptr<Esp32TrainDbEntry> &train)
    {
      return train->get_legacy_address() == (uint16_t)address;
    });
  if (entry != knownTrains_.end())
  {
    LOG(VERBOSE, "[TrainDB] Setting auto-idle: %s"
      , idle ? JSON_VALUE_ON : JSON_VALUE_OFF);
    (*entry)->set_auto_idle(idle);
  }
  else
  {
    LOG_ERROR("[TrainDB] train %u not found, unable to set idle state!"
            , address);
  }
}

void Esp32TrainDatabase::set_train_show_on_limited_throttle(unsigned address
                                                          , bool show)
{
  OSMutexLock l(&knownTrainsLock_);
  LOG(VERBOSE, "[TrainDB] Searching for train with address %u", address);
  auto entry = std::find_if(knownTrains_.begin(), knownTrains_.end(),
    [address](const shared_ptr<Esp32TrainDbEntry> &train)
    {
      return train->get_legacy_address() == (uint16_t)address;
    });
  if (entry != knownTrains_.end())
  {
    LOG(VERBOSE, "[TrainDB] Setting visible on limited throttes: %s"
      , show ? JSON_VALUE_ON : JSON_VALUE_OFF);
    (*entry)->set_show_on_limited_throttles(show);
  }
  else
  {
    LOG_ERROR("[TrainDB] train %u not found, unable to set limited throttle!"
            , address);
  }
}

string Esp32TrainDatabase::get_all_entries_as_json()
{
  OSMutexLock l(&knownTrainsLock_);
  // If we don't have any trains in our db return an empty set to the caller.
  if (knownTrains_.empty())
  {
    return "[]";
  }
  // convert our db into a json set and return it to the caller.
  json j;
  for (auto entry : knownTrains_)
  {
    j.push_back(entry->get_data());
  }
  return j.dump();
}

string Esp32TrainDatabase::get_entry_as_json(unsigned address)
{
  OSMutexLock l(&knownTrainsLock_);
  auto entry = std::find_if(knownTrains_.begin(), knownTrains_.end(),
    [address](const shared_ptr<Esp32TrainDbEntry> &train)
    {
      return train->get_legacy_address() == (uint16_t)address;
    });
  if (entry != knownTrains_.end())
  {
    json j = (*entry)->get_data();
    return j.dump();
  }
  return "{}";
}

void Esp32TrainDatabase::persist()
{
  OSMutexLock l(&knownTrainsLock_);
  LOG(VERBOSE, "[TrainDB] Checking if roster needs to be persisted...");
  auto ent = std::find_if(knownTrains_.begin(), knownTrains_.end(),
    [](const shared_ptr<Esp32TrainDbEntry> &train)
    {
      return train->is_dirty() && train->is_persisted();
    });
  if (ent != knownTrains_.end() || legacyEntriesFound_)
  {
    LOG(VERBOSE, "[TrainDB] At least one entry requires persistence.");
    json j;
    size_t count = 0;
    for (auto entry : knownTrains_)
    {
      if (entry->is_persisted())
      {
        j.push_back(entry->get_data());
        count++;
      }
      entry->reset_dirty();
    }
    Singleton<ConfigurationManager>::instance()->store(TRAIN_DB_JSON_FILE
                                                     , j.dump());
    LOG(INFO, "[TrainDB] Persisted %zu entries.", count);

    // if we loaded legacy entries we need to clean up the old file
    if (legacyEntriesFound_)
    {
      Singleton<ConfigurationManager>::instance()->remove(LEGACY_ROSTER_JSON_FILE);
    }
    legacyEntriesFound_ = false;
  }
  else
  {
    LOG(VERBOSE, "[TrainDB] No entries require persistence");
  }
}

} // namespace esp32cs
