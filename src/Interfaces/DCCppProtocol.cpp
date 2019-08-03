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

The DCC++ protocol specification is
COPYRIGHT (c) 2013-2016 Gregg E. Berman
and has been adapter for use in ESP32 COMMAND STATION.

**********************************************************************/

#include "ESP32CommandStation.h"

LinkedList<DCCPPProtocolCommand *> registeredCommands([](DCCPPProtocolCommand *command) {delete command; });

// <e> command handler, this command will clear all stored configuration data
// on the ESP32. All Turnouts, Outputs, Sensors and S88 Sensors (if enabled)
// will need to be reconfigured after sending this command.
class ConfigErase : public DCCPPProtocolCommand {
public:
  void process(const vector<string> arguments) {
    configStore->clear();
    turnoutManager->clear();
#if SENSORS_ENABLED
    SensorManager::clear();
#if S88_ENABLED
    S88BusManager::clear();
#endif
#endif
#if ENABLE_OUTPUTS
    OutputManager::clear();
#endif
    LocomotiveManager::clear();
    wifiInterface.broadcast(COMMAND_SUCCESSFUL_RESPONSE);
  }

  string getID() {
    return "e";
  }
};

// <E> command handler, this command stores all currently defined Turnouts,
// Sensors, S88 Sensors (if enabled), Outputs and locomotives into the ESP32 for use on
// subsequent startups.
class ConfigStore : public DCCPPProtocolCommand {
public:
  void process(const vector<string> arguments) {
#if S88_ENABLED
    wifiInterface.broadcast(StringPrintf("<e %d %d %d %d %d>"),
      TurnoutManager::store(),
      SensorManager::store(),
      OutputManager::store(),
      S88BusManager::store(),
      LocomotiveManager::store()));
#else
    wifiInterface.broadcast(StringPrintf("<e %d %d %d 0 %d>",
      turnoutManager->store(),
#if SENSORS_ENABLED
      SensorManager::store(),
#else
      0,
#endif
#if ENABLE_OUTPUTS
      OutputManager::store(),
#else
      0,
#endif
      LocomotiveManager::store()));
#endif
  }

  string getID() {
    return "E";
  }
};

// <R {CV} {CALLBACK} {CALLBACK-SUB}> command handler, this command attempts
// to read a CV value from the PROGRAMMING track. The returned value will be
// the actual CV value or -1 when there is a failure reading or verifying the CV.
class ReadCVCommand : public DCCPPProtocolCommand {
public:
  void process(const vector<string> arguments) {
    uint16_t cv = std::stoi(arguments[0]);
    uint16_t callback = std::stoi(arguments[1]);
    uint16_t callbackSub = std::stoi(arguments[2]);
    int16_t value = readCV(cv);
    wifiInterface.broadcast(StringPrintf("<r%d|%d|%d %d>",
      callback,
      callbackSub,
      cv,
      value));
  }

  string getID() {
    return "R";
  }
};

// <W {CV} {VALUE} {CALLBACK} {CALLBACK-SUB}> command handler, this command
// attempts to write a CV value on the PROGRAMMING track. The returned value
// is either the actual CV value written or -1 if there is a failure writing or
// verifying the CV value.
class WriteCVByteProgCommand : public DCCPPProtocolCommand {
public:
  void process(const vector<string> arguments) {
    uint16_t cv = std::stoi(arguments[0]);
    uint16_t value = std::stoi(arguments[1]);
    uint16_t callback = std::stoi(arguments[2]);
    uint16_t callbackSub = std::stoi(arguments[3]);
    if (!writeProgCVByte(cv, value))
    {
      LOG(INFO, "[PROG] Failed to write CV %d as %d", cv, value);
      wifiInterface.broadcast(StringPrintf("<r%d|%d|%d -1>", callback, callbackSub, cv));
    }
    else
    {
      LOG(INFO, "[PROG] CV %d set and verify as %d", cv, value);
      wifiInterface.broadcast(StringPrintf("<r%d|%d|%d %d>", callback, callbackSub, cv, value));
    }
  }

  string getID() {
    return "W";
  }
};

// <W {CV} {BIT} {VALUE} {CALLBACK} {CALLBACK-SUB}> command handler, this
// command attempts to write a single bit value for a CV on the PROGRAMMING
// track. The returned value is either the actual bit value of the CV or -1 if
// there is a failure writing or verifying the CV value.
class WriteCVBitProgCommand : public DCCPPProtocolCommand {
public:
  void process(const vector<string> arguments) {
    int cv = std::stoi(arguments[0]);
    uint8_t bit = std::stoi(arguments[1]);
    int8_t value = std::stoi(arguments[2]);
    uint16_t callback = std::stoi(arguments[3]);
    uint16_t callbackSub = std::stoi(arguments[4]);
    if (!writeProgCVBit(cv, bit, value))
    {
      LOG(INFO, "[PROG] Failed to write CV %d BIT %d as %d", cv, bit, value);
      wifiInterface.broadcast(StringPrintf("<r%d|%d|%d %d -1>", callback, callbackSub, cv, bit));
    }
    else
    {
      wifiInterface.broadcast(StringPrintf("<r%d|%d|%d %d %d>", callback, callbackSub, cv, bit, value));
    }
  }

  string getID() {
    return "B";
  }
};

// <w {LOCO} {CV} {VALUE}> command handler, this command sends a CV write packet
// on the MAIN OPERATIONS track for a given LOCO. No verification is attempted.
class WriteCVByteOpsCommand : public DCCPPProtocolCommand {
public:
  void process(const vector<string> arguments) {
    writeOpsCVByte(std::stoi(arguments[0]),
      std::stoi(arguments[1]),
      std::stoi(arguments[2]));
  }

  string getID() {
    return "w";
  }
};

// <w {LOCO} {CV} {BIT} {VALUE}> command handler, this command sends a CV bit
// write packet on the MAIN OPERATIONS track for a given LOCO. No verification
// is attempted.
class WriteCVBitOpsCommand : public DCCPPProtocolCommand {
public:
  void process(const vector<string> arguments) {
    writeOpsCVBit(std::stoi(arguments[0]),
      std::stoi(arguments[1]),
      std::stoi(arguments[2]),
      arguments[3][0] == '1');
  }

  string getID() {
    return "b";
  }
};

// <s> command handler, this command sends the current status for all parts of
// the ESP32 Command Station. JMRI uses this command as a keep-alive heartbeat
// command.
class StatusCommand : public DCCPPProtocolCommand {
public:
  void process(const vector<string> arguments) {
    wifiInterface.broadcast(
      StringPrintf("<iDCC++ ESP32 Command Station: V-%s / %s %s>"
                 , ESP32CS_VERSION
                 , __DATE__
                 , __TIME__));
    trackSignal->broadcast_status();
    LocomotiveManager::showStatus();
    turnoutManager->showStatus();
#if ENABLE_OUTPUTS
    OutputManager::showStatus();
#endif
    wifiInterface.showInitInfo();
  }

  string getID() {
    return "s";
  }
};

// <F> command handler, this command sends the current free heap space as response.
class FreeHeapCommand : public DCCPPProtocolCommand {
public:
  void process(const vector<string> arguments) {
    wifiInterface.broadcast(StringPrintf("<f %d>", ESP.getFreeHeap()));
  }

  string getID() {
    return "F";
  }
};

// <estop> command handler, this command sends the current free heap space as response.
class EStopCommand : public DCCPPProtocolCommand {
public:
  void process(const vector<string> arguments) {
    LocomotiveManager::emergencyStop();
  }

  string getID() {
    return "estop";
  }
};

class CurrentDrawCommand : public DCCPPProtocolCommand {
public:
  void process(const vector<string> arguments)
  {
    trackSignal->broadcast_status();
  }
  string getID()
  {
    return "c";
  }
};

class PowerOnCommand : public DCCPPProtocolCommand {
public:
  void process(const vector<string> arguments)
  {
    trackSignal->enable_ops_output();
  }
  string getID() {
    return "1";
  }
};

class PowerOffCommand : public DCCPPProtocolCommand {
public:
  void process(const vector<string> arguments)
  {
    trackSignal->disable_ops_output();
  }
  string getID() {
    return "0";
  }
};

void DCCPPProtocolHandler::init() {
  registerCommand(new ThrottleCommandAdapter());
  registerCommand(new ThrottleExCommandAdapter());
  registerCommand(new FunctionCommandAdapter());
  registerCommand(new FunctionExCommandAdapter());
  registerCommand(new ConsistCommandAdapter());
  registerCommand(new AccessoryCommand());
  registerCommand(new PowerOnCommand());
  registerCommand(new PowerOffCommand());
  registerCommand(new CurrentDrawCommand());
  registerCommand(new StatusCommand());
  registerCommand(new ReadCVCommand());
  registerCommand(new WriteCVByteProgCommand());
  registerCommand(new WriteCVBitProgCommand());
  registerCommand(new WriteCVByteOpsCommand());
  registerCommand(new WriteCVBitOpsCommand());
  registerCommand(new ConfigErase());
  registerCommand(new ConfigStore());
#if OUTPUTS_ENABLED
  registerCommand(new OutputCommandAdapter());
  registerCommand(new OutputExCommandAdapter());
#endif
  registerCommand(new TurnoutCommandAdapter());
  registerCommand(new TurnoutExCommandAdapter());
#if SENSORS_ENABLED
  registerCommand(new SensorCommandAdapter());
#if S88_ENABLED
  registerCommand(new S88BusCommandAdapter());
#endif
  registerCommand(new RemoteSensorsCommandAdapter());
#endif
  registerCommand(new FreeHeapCommand());
  registerCommand(new EStopCommand());
}

void DCCPPProtocolHandler::process(const string &commandString) {
  vector<string> parts;
  std::stringstream buf(commandString);
  string part;
  while(getline(buf, part, ' ')) {
    parts.push_back(part);
  }
  string commandID = parts.front();
  parts.erase(parts.begin());
  LOG(VERBOSE, "Command: %s, argument count: %d", commandID.c_str(), parts.size());
  bool processed = false;
  for (const auto& command : registeredCommands) {
    if(commandID == command->getID()) {
      command->process(parts);
      processed = true;
    }
  }
  if(!processed) {
    LOG_ERROR("No command handler for [%s]", commandID.c_str());
    wifiInterface.broadcast(COMMAND_FAILED_RESPONSE);
  }
}

void DCCPPProtocolHandler::registerCommand(DCCPPProtocolCommand *cmd) {
  for (const auto& command : registeredCommands) {
    if(!command->getID().compare(cmd->getID())) {
      LOG_ERROR("Ignoring attempt to register second command with ID: %s",
        cmd->getID().c_str());
      return;
    }
  }
  LOG(VERBOSE, "Registering interface command %s", cmd->getID().c_str());
  registeredCommands.add(cmd);
}

DCCPPProtocolCommand *DCCPPProtocolHandler::getCommandHandler(const std::string &id) {
  for (const auto& command : registeredCommands) {
    if(command->getID() == id) {
      return command;
    }
  }
  return nullptr;
}

DCCPPProtocolConsumer::DCCPPProtocolConsumer() {
  _buffer.reserve(256);
}

void DCCPPProtocolConsumer::feed(uint8_t *data, size_t len) {
  for(size_t i = 0; i < len; i++) {
    _buffer.emplace_back(data[i]);
  }
  processData();
}

void DCCPPProtocolConsumer::processData() {
  auto s = _buffer.begin();
  auto consumed = _buffer.begin();
  for(; s != _buffer.end();) {
    s = std::find(s, _buffer.end(), '<');
    auto e = std::find(s, _buffer.end(), '>');
    if(s != _buffer.end() && e != _buffer.end()) {
      // discard the <
      s++;
      // discard the >
      *e = 0;
      std::string str(reinterpret_cast<char*>(&*s));
      DCCPPProtocolHandler::process(std::move(str));
      consumed = e;
    }
    s = e;
  }
  _buffer.erase(_buffer.begin(), consumed); // drop everything we used from the buffer.
}

