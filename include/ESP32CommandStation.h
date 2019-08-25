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

#ifndef ESP32_CS_H_
#define ESP32_CS_H_

#include <algorithm>
#include <functional>
#include <string>
#include <sstream>
#include <vector>
#include <memory>
#include <set>

using std::unique_ptr;
using std::shared_ptr;
using std::vector;
using std::string;
using std::set;

// ESP-IDF includes
#include <driver/uart.h>
#include <esp_ota_ops.h>

// disable Arduino-esp32 binary.h inclusion as it conflicts with
// esp_vfs.h/termios.h
#define Binary_h
#include <Arduino.h>

// Arduino libraries
#include <ESPAsyncWebServer.h>

// get rid of the arduino-esp32 abs override
#undef abs
// include json library
#include <json.hpp>

// OpenMRN library components
#include <OpenMRNLite.h>

#include <dcc/DccDebug.hxx>
#include <dcc/LocalTrackIf.hxx>
#include <dcc/Loco.hxx>
#include <dcc/Packet.hxx>
#include <dcc/PacketFlowInterface.hxx>
#include <dcc/ProgrammingTrackBackend.hxx>
#include <dcc/RailcomHub.hxx>
#include <dcc/RailcomPortDebug.hxx>
#include <dcc/SimpleUpdateLoop.hxx>

#include <executor/PoolToQueueFlow.hxx>

#include <openlcb/ConfiguredTcpConnection.hxx>
#include <openlcb/DccAccyConsumer.hxx>
#include <openlcb/DccAccyProducer.hxx>
#include <openlcb/MemoryConfig.hxx>
#include <openlcb/SimpleInfoProtocol.hxx>
#include <openlcb/TcpDefs.hxx>
#include <openlcb/TractionCvSpace.hxx>
#include <openlcb/TractionProxy.hxx>
#include <openlcb/TractionTrain.hxx>

#include <os/MDNS.hxx>

#include <utils/AutoSyncFileFlow.hxx>
#include <utils/constants.hxx>
#include <utils/format_utils.hxx>
#include <utils/logging.h>
#include <utils/macros.h>
#include <utils/StringPrintf.hxx>

// Train Search components
#include "TrainDb.hxx"
#include "AllTrainNodes.hxx"

// Define NOT_A_PIN in case it hasn't been defined already, this should be
// defined inside Arduino.h
#ifndef NOT_A_PIN
#define NOT_A_PIN -1
#endif

// Define NOT_A_PORT in case it hasn't been defined already, this should be
// defined inside Arduino.h
#ifndef NOT_A_PORT
#define NOT_A_PORT -1
#endif

// include user supplied configuration settings if we are not building using
// an externally supplied configuration set (ie: pcb build settings)
#ifndef ESP32CS_EXTERNAL_CONFIGURATION
#include "Config.h"
#endif

// After including the user supplied configuration or externally supplied
// configuration we need to set defaults for anything that was not explicitly
// configured.
#include "DefaultConfigs.h"

// Sanity check the configuration and generate a compilation failure if any of
// the settings overlap or are invalid.
#include "ConfigValidation.h"

// Declare namespace uses for OpenMRN components that are used
using dcc::Dcc128Train;
using dcc::Dcc28Train;
using dcc::DccLongAddress;
using dcc::DccShortAddress;
using dcc::LocalTrackIf;
using dcc::Packet;
using dcc::PacketFlowInterface;
using dcc::RailcomHubFlow;
using dcc::RailcomPrintfFlow;
using dcc::SimpleUpdateLoop;
using dcc::SpeedType;

using openlcb::BitEventProducer;
using openlcb::DccAccyConsumer;
using openlcb::Defs;
using openlcb::EventId;
using openlcb::EventRegistry;
using openlcb::EventRegistryEntry;
using openlcb::EventReport;
using openlcb::EventState;
using openlcb::MemoryBit;
using openlcb::MemoryConfigDefs;
using openlcb::MemoryConfigHandler;
using openlcb::Node;
using openlcb::NodeID;
using openlcb::ROFileMemorySpace;
using openlcb::SimpleCanStack;
using openlcb::SimpleInfoFlow;
using openlcb::TractionCvSpace;
using openlcb::TractionDefs;
using openlcb::TrainImpl;
using openlcb::TrainNodeForProxy;
using openlcb::TractionProxyService;
using openlcb::TrainService;
using openlcb::WriteHelper;

// Include ESP32 Command Station component declarations
#include "ESP32CSConstants.h"
#include "JsonConstants.h"
#include "ConfigurationManager.h"
#include "ESP32TrainDatabase.h"

#include "dcc/DCCProgrammer.h"
#include "dcc/EStopHandler.h"
#include "dcc/Locomotive.h"
#include "dcc/MonitoredHBridge.h"
#include "dcc/RMTTrackDevice.h"
#include "dcc/Turnouts.h"

#include "interfaces/DCCppProtocol.h"
#include "interfaces/NextionInterface.h"
#include "interfaces/WiFiInterface.h"

#include "stateflows/FreeRTOSTaskMonitor.h"
#include "stateflows/InfoScreen.h"
#include "stateflows/LCCStatCollector.h"
#include "stateflows/StatusLED.h"
#include "stateflows/HC12Radio.h"
#include "stateflows/OTAMonitor.h"

#include "io/Outputs.h"
#include "io/Sensors.h"
#include "io/S88Sensors.h"
#include "io/RemoteSensors.h"

extern unique_ptr<RMTTrackDevice> trackSignal;
extern unique_ptr<LocalTrackIf> trackInterface;
extern unique_ptr<commandstation::AllTrainNodes> trainNodes;

#if LOCONET_ENABLED
#include <LocoNetESP32UART.h>
extern LocoNetESP32Uart locoNet;
#endif

void initializeLocoNet();

// Returns true if the provided pin is one of the ESP32 pins that has usage
// restrictions. This will always return false if the configuration flag
// ALLOW_USAGE_OF_RESTRICTED_GPIO_PINS is enabled.
bool is_restricted_pin(int8_t);

#endif // ESP32_CS_H_