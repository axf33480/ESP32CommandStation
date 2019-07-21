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
    SensorManager::clear();
#if S88_ENABLED
    S88BusManager::clear();
#endif
    OutputManager::clear();
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
      SensorManager::store(),
      OutputManager::store(),
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
    int cvNumber = std::stoi(arguments[0]);
    int16_t cvValue = -1;
    if(enterProgrammingMode()) {
      cvValue = readCV(cvNumber);
      leaveProgrammingMode();
    }
    wifiInterface.broadcast(StringPrintf("<r%s|%s|%d %d>",
      arguments[1].c_str(),
      arguments[2].c_str(),
      cvNumber,
      cvValue));
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
    int cvNumber = std::stoi(arguments[0]);
    int16_t cvValue = std::stoi(arguments[1]);
    if(enterProgrammingMode()) {
      if(!writeProgCVByte(cvNumber, cvValue)) {
        cvValue = -1;
      }
      leaveProgrammingMode();
    } else {
      cvValue = -1;
    }
    wifiInterface.broadcast(StringPrintf("<r%s|%s|%d %d>",
      arguments[2].c_str(),
      arguments[3].c_str(),
      cvNumber,
      cvValue));
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
    int cvNumber = std::stoi(arguments[0]);
    uint8_t bit = std::stoi(arguments[1]);
    int8_t bitValue = std::stoi(arguments[1]);
    if(enterProgrammingMode()) {
      if(!writeProgCVBit(cvNumber, bit, bitValue == 1)) {
        bitValue = -1;
      }
      leaveProgrammingMode();
    } else {
      bitValue = -1;
    }
    wifiInterface.broadcast(StringPrintf("<r%s|%s|%d %d %d>",
      arguments[2].c_str(),
      arguments[3].c_str(),
      cvNumber,
      bit,
      bitValue));
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
    wifiInterface.broadcast(StringPrintf("<iDCC++ ESP32 Command Station: V-%s / %s %s>",
      VERSION, __DATE__, __TIME__));
    broadcast_all_hbridge_statuses();
    LocomotiveManager::showStatus();
    turnoutManager->showStatus();
    OutputManager::showStatus();
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
    if (arguments.empty())
    {
      broadcast_all_hbridge_statuses();
    }
    else
    {
      broadcast_named_hbridge_status(arguments[0]);
    }
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
    if (arguments.empty())
    {
      enable_all_hbridges();
    }
    else
    {
      enable_named_hbridge(arguments[0]);
    }
  }
  string getID() {
    return "1";
  }
};

class PowerOffCommand : public DCCPPProtocolCommand {
public:
  void process(const vector<string> arguments)
  {
    if (arguments.empty())
    {
      disable_all_hbridges();
    }
    else
    {
      disable_named_hbridge(arguments[0]);
    }
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
  registerCommand(new OutputCommandAdapter());
  registerCommand(new OutputExCommandAdapter());
  registerCommand(new TurnoutCommandAdapter());
  registerCommand(new TurnoutExCommandAdapter());
  registerCommand(new SensorCommandAdapter());
#if defined(S88_ENABLED) && S88_ENABLED
  registerCommand(new S88BusCommandAdapter());
#endif
  registerCommand(new RemoteSensorsCommandAdapter());
  registerCommand(new FreeHeapCommand());
  registerCommand(new EStopCommand());
}

void DCCPPProtocolHandler::process(const std::string &commandString) {
  std::vector<std::string> parts;
  std::stringstream buf(commandString);
  std::string part;
  while(getline(buf, part, ' ')) {
    parts.push_back(part);
  }
  std::string commandID = parts.front();
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
