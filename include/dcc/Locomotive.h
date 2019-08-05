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

#ifndef LOCOMOTIVE_H_
#define LOCOMOTIVE_H_

#include <dcc/Loco.hxx>
#include <openlcb/TractionTrain.hxx>

#include "interfaces/DCCppProtocol.h"
#include "SimplifiedCallbackEventHandler.h"

#define MAX_LOCOMOTIVE_FUNCTIONS 29
#define MAX_LOCOMOTIVE_FUNCTION_PACKETS 5

class Locomotive : public dcc::Dcc128Train
                 , public openlcb::TrainNodeForProxy
{
public:
  Locomotive(uint16_t, openlcb::TrainService *);

  virtual ~Locomotive()
  {
    service_->unregister_train(this);
    LOG(INFO, "[Loco %d] Deleted", legacy_address());
  }

  void setRegister(int8_t registerNumber)
  {
    _registerNumber = registerNumber;
  }

  int8_t getRegister()
  {
    return _registerNumber;
  }

  void set_speed(dcc::SpeedType speed) override
  {
    dcc::Dcc128Train::set_speed(speed);
    showStatus();
  }

  void set_fn(uint32_t address, uint16_t value) override
  {
    dcc::Dcc128Train::set_fn(address, value);
    LOG(INFO, "[Loco %d] Set function %d to %s", legacy_address(), address
      , value ? JSON_VALUE_ON : JSON_VALUE_OFF);
  }

  void setOrientationForward(bool forward)
  {
    _orientation = forward;
  }

  bool isOrientationForward()
  {
    return _orientation;
  }

  void showStatus();
  void toJson(JsonObject, bool=true);
  static Locomotive *fromJson(JsonObject, openlcb::TrainService *);
  static Locomotive *fromJsonFile(const char *, openlcb::TrainService *);
private:
  int8_t _registerNumber{-1};
  bool _orientation{true};
};

class LocomotiveConsist : public Locomotive
{
public:
  LocomotiveConsist(uint8_t address
                  , openlcb::TrainService *trainService
                  , bool decoderAssistedConsist=false)
                  : Locomotive(address, trainService)
                  , _decoderAssisstedConsist(decoderAssistedConsist)
  {
  }
  virtual ~LocomotiveConsist();
  void showStatus();
  bool isAddressInConsist(uint16_t);
  void updateThrottle(uint16_t, int8_t, bool);
  void addLocomotive(uint16_t, bool, uint8_t);
  bool removeLocomotive(uint16_t);
  void releaseLocomotives();
  bool isDecoderAssistedConsist()
  {
    return _decoderAssisstedConsist;
  }
  void toJson(JsonObject, const bool=true);
  static LocomotiveConsist *fromJson(JsonObject, openlcb::TrainService *);
  static LocomotiveConsist *fromJsonFile(const char *, openlcb::TrainService *);
private:
  bool _decoderAssisstedConsist;
  std::vector<Locomotive *> _locos;
};

class RosterEntry {
public:
  RosterEntry(uint16_t address)
    : _description("")
    , _address(address)
    , _type("")
    , _idleOnStartup(false)
    , _defaultOnThrottles(false)
  {
  }
  RosterEntry(JsonObject);
  RosterEntry(const char *);
  virtual ~RosterEntry();
  void toJson(JsonObject);
  void setDescription(std::string description)
  {
    _description = description;
  }
  std::string getDescription()
  {
    return _description;
  }
  void setAddress(const uint16_t address)
  {
    _address = address;
  }
  uint16_t getAddress() {
    return _address;
  }
  void setType(std::string type)
  {
    _type = type;
  }
  std::string getType() {
    return _type;
  }
  void setIdleOnStartup(bool value=false)
  {
    _idleOnStartup = value;
  }
  bool isIdleOnStartup()
  {
    return _idleOnStartup;
  }
  void setDefaultOnThrottles(bool value=false)
  {
    _defaultOnThrottles = value;
  }
  bool isDefaultOnThrottles()
  {
    return _defaultOnThrottles;
  }

private:
  std::string _description;
  uint16_t _address;
  std::string _type;
  bool _idleOnStartup;
  bool _defaultOnThrottles;
};

class LocomotiveManager : public openlcb::BitEventInterface
                        , private Atomic
{
public:
  LocomotiveManager(openlcb::Node *, openlcb::TrainService *);
  // gets or creates a new locomotive to be managed
  Locomotive *getLocomotive(const uint16_t, const bool=true);
  Locomotive *getLocomotiveByRegister(const uint8_t);
  // removes a locomotive from management, sends speed zero before removal
  void removeLocomotive(const uint16_t);
  bool removeLocomotiveConsist(const uint16_t);
  void processThrottle(const std::vector<std::string>);
  void processThrottleEx(const std::vector<std::string>);
  void processFunction(const std::vector<std::string>);
  void processFunctionEx(const std::vector<std::string>);
  void processConsistThrottle(const std::vector<std::string>);
  void showStatus();
  void showConsistStatus();
  uint8_t getActiveLocoCount()
  {
    return locos_.size();
  }
  void clear();
  uint16_t store();
  std::vector<RosterEntry *> getDefaultLocos(const int8_t=-1);
  void getDefaultLocos(JsonArray);
  void getActiveLocos(JsonArray);
  void getRosterEntries(JsonArray);
  bool isConsistAddress(uint16_t);
  bool isAddressInConsist(uint16_t);
  LocomotiveConsist *getConsistByID(uint8_t);
  LocomotiveConsist *getConsistForLoco(uint16_t);
  LocomotiveConsist *createLocomotiveConsist(int8_t);
  RosterEntry *getRosterEntry(uint16_t, bool=true);
  void removeRosterEntry(uint16_t);
  // BitEventInterface method to return current track output status.
  openlcb::EventState get_current_state() override
  {
    return EventState::INVALID; // is_enabled() ? EventState::VALID : EventState::INVALID;
  }

  // BitEventInterface method to enable/disable track output.
  void set_state(bool new_value) override
  {
    if (new_value)
    {
      for (const auto& loco : locos_)
      {
        loco->set_emergencystop();
      }
    }
    else
    {
      // TBD
    }
  }

  // BitEventInterface method
  Node *node() override
  {
    return node_;
  }
private:
  std::vector<std::unique_ptr<Locomotive>> locos_;
  std::vector<std::unique_ptr<LocomotiveConsist>> consists_;
  std::vector<std::unique_ptr<RosterEntry>> roster_;
  openlcb::Node *node_;
  openlcb::TrainService *trainService_;
};

extern std::unique_ptr<LocomotiveManager> locoManager;

// <t {REGISTER} {LOCO} {SPEED} {DIRECTION}> command handler, this command
// converts the provided locomotive control command into a compatible DCC
// locomotive control packet.
DECLARE_DCC_PROTOCOL_COMMAND_CLASS(ThrottleCommandAdapter, "t")

// <tex {LOCO} {SPEED} {DIRECTION}> command handler, this command
// converts the provided locomotive control command into a compatible DCC
// locomotive control packet.
DECLARE_DCC_PROTOCOL_COMMAND_CLASS(ThrottleExCommandAdapter, "tex")

// <f {LOCO} {BYTE} [{BYTE2}]> command handler, this command converts a
// locomotive function update into a compatible DCC function control packet.
DECLARE_DCC_PROTOCOL_COMMAND_CLASS(FunctionCommandAdapter, "f")

// <fex {LOCO} {FUNC} {STATE}]> command handler, this command converts a
// locomotive function update into a compatible DCC function control packet.
DECLARE_DCC_PROTOCOL_COMMAND_CLASS(FunctionExCommandAdapter, "fex")

// wrapper to handle the following command structures:
// CREATE: <C {ID} {LEAD LOCO} {TRAIL LOCO}  [{OTHER LOCO}]>
// DELETE: <C {ID} {LOCO}>
// DELETE: <C {ID}>
// QUERY : <C 0 {LOCO}>
// SHOW  : <C>
DECLARE_DCC_PROTOCOL_COMMAND_CLASS(ConsistCommandAdapter, "C")

#endif // LOCOMOTIVE_H_