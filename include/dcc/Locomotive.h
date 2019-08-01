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

#include "interfaces/DCCppProtocol.h"
#include "SimplifiedCallbackEventHandler.h"

#define MAX_LOCOMOTIVE_FUNCTIONS 29
#define MAX_LOCOMOTIVE_FUNCTION_PACKETS 5

class Locomotive : public dcc::Dcc128Train
{
public:
  Locomotive(uint16_t, const bool=true);
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

  void setOrientationForward(bool forward) {
    _orientation = forward;
  }

  bool isOrientationForward() {
    return _orientation;
  }
  void showStatus();
  void toJson(JsonObject, bool=true, bool=true);
  static Locomotive *fromJson(JsonObject, bool=true);
  static Locomotive *fromJsonFile(const char *, bool=true);
private:
  int8_t _registerNumber{-1};
  bool _orientation{true};
};

class LocomotiveConsist : public Locomotive
{
public:
  LocomotiveConsist(uint8_t address, bool decoderAssistedConsist=false) :
    Locomotive(address), _decoderAssisstedConsist(decoderAssistedConsist) {
  }
  virtual ~LocomotiveConsist();
  void showStatus();
  bool isAddressInConsist(uint16_t);
  void updateThrottle(uint16_t, int8_t, bool);
  void addLocomotive(uint16_t, bool, uint8_t);
  bool removeLocomotive(uint16_t);
  void releaseLocomotives();
  bool isDecoderAssistedConsist() {
    return _decoderAssisstedConsist;
  }
  void toJson(JsonObject, bool=true, const bool=true);
  static LocomotiveConsist *fromJson(JsonObject);
  static LocomotiveConsist *fromJsonFile(const char *);
private:
  bool _decoderAssisstedConsist;
  std::vector<Locomotive *> _locos;
};

class RosterEntry {
public:
  RosterEntry(uint16_t address) : _description(""), _address(address), _type(""),
    _idleOnStartup(false), _defaultOnThrottles(false) {}
  RosterEntry(JsonObject);
  RosterEntry(const char *);
  void toJson(JsonObject);
  void setDescription(std::string description) {
    _description = description;
  }
  std::string getDescription() {
    return _description;
  }
  void setAddress(const uint16_t address) {
    _address = address;
  }
  uint16_t getAddress() {
    return _address;
  }
  void setType(std::string type) {
    _type = type;
  }
  std::string getType() {
    return _type;
  }
  void setIdleOnStartup(bool value=false) {
    _idleOnStartup = value;
  }
  bool isIdleOnStartup() {
    return _idleOnStartup;
  }
  void setDefaultOnThrottles(bool value=false) {
    _defaultOnThrottles = value;
  }
  bool isDefaultOnThrottles() {
    return _defaultOnThrottles;
  }

private:
  std::string _description;
  uint16_t _address;
  std::string _type;
  bool _idleOnStartup;
  bool _defaultOnThrottles;
};

class LocomotiveManager {
public:
  // gets or creates a new locomotive to be managed
  static Locomotive *getLocomotive(const uint16_t, const bool=true);
  static Locomotive *getLocomotiveByRegister(const uint8_t);
  // removes a locomotive from management, sends speed zero before removal
  static void removeLocomotive(const uint16_t);
  static bool removeLocomotiveConsist(const uint16_t);
  static void processThrottle(const std::vector<std::string>);
  static void processThrottleEx(const std::vector<std::string>);
  static void processFunction(const std::vector<std::string>);
  static void processFunctionEx(const std::vector<std::string>);
  static void processConsistThrottle(const std::vector<std::string>);
  static void showStatus();
  static void showConsistStatus();
  static void emergencyStop();
  static uint8_t getActiveLocoCount() {
    return _locos.length();
  }
  static void init(openlcb::Node *node);
  static void clear();
  static uint16_t store();
  static std::vector<RosterEntry *> getDefaultLocos(const int8_t=-1);
  static void getDefaultLocos(JsonArray);
  static void getActiveLocos(JsonArray);
  static void getRosterEntries(JsonArray);
  static bool isConsistAddress(uint16_t);
  static bool isAddressInConsist(uint16_t);
  static LocomotiveConsist *getConsistByID(uint8_t);
  static LocomotiveConsist *getConsistForLoco(uint16_t);
  static LocomotiveConsist *createLocomotiveConsist(int8_t);
  static RosterEntry *getRosterEntry(uint16_t, bool=true);
  static void removeRosterEntry(uint16_t);
private:
  static LinkedList<RosterEntry *> _roster;
  static LinkedList<Locomotive *> _locos;
  static LinkedList<LocomotiveConsist *> _consists;
  static unique_ptr<SimplifiedCallbackEventHandler> _eStopCallback;
};

// <t {REGISTER} {LOCO} {SPEED} {DIRECTION}> command handler, this command
// converts the provided locomotive control command into a compatible DCC
// locomotive control packet.
class ThrottleCommandAdapter : public DCCPPProtocolCommand {
public:
  void process(const std::vector<std::string> arguments) {
    LocomotiveManager::processThrottle(arguments);
  }
  std::string getID() {
    return "t";
  }
};

// <tex {LOCO} {SPEED} {DIRECTION}> command handler, this command
// converts the provided locomotive control command into a compatible DCC
// locomotive control packet.
class ThrottleExCommandAdapter : public DCCPPProtocolCommand {
public:
  void process(const std::vector<std::string> arguments) {
    LocomotiveManager::processThrottleEx(arguments);
  }
  std::string getID() {
    return "tex";
  }
};

// <f {LOCO} {BYTE} [{BYTE2}]> command handler, this command converts a
// locomotive function update into a compatible DCC function control packet.
class FunctionCommandAdapter : public DCCPPProtocolCommand {
public:
  void process(const std::vector<std::string> arguments) {
    LocomotiveManager::processFunction(arguments);
  }
  std::string getID() {
    return "f";
  }
};

// <fex {LOCO} {FUNC} {STATE}]> command handler, this command converts a
// locomotive function update into a compatible DCC function control packet.
class FunctionExCommandAdapter : public DCCPPProtocolCommand {
public:
  void process(const std::vector<std::string> arguments) {
    LocomotiveManager::processFunctionEx(arguments);
  }
  std::string getID() {
    return "fex";
  }
};

// wrapper to handle the following command structures:
// CREATE: <C {ID} {LEAD LOCO} {TRAIL LOCO}  [{OTHER LOCO}]>
// DELETE: <C {ID} {LOCO}>
// DELETE: <C {ID}>
// QUERY : <C 0 {LOCO}>
// SHOW  : <C>
class ConsistCommandAdapter : public DCCPPProtocolCommand {
public:
  void process(const std::vector<std::string>);
  std::string getID() {
    return "C";
  }
};

#endif // LOCOMOTIVE_H_