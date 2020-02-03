/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2017-2020 Mike Dunston

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

// include json library
#include <json.hpp>

// OpenMRN library components
//#include <OpenMRNLite.h>

#include <dcc/DccDebug.hxx>
#include <dcc/LocalTrackIf.hxx>
#include <dcc/Loco.hxx>
#include <dcc/Packet.hxx>
#include <dcc/PacketFlowInterface.hxx>
#include <dcc/ProgrammingTrackBackend.hxx>
#include <dcc/RailcomHub.hxx>
#include <dcc/RailcomPortDebug.hxx>
#include <dcc/SimpleUpdateLoop.hxx>

#include <esp_image_format.h>

#include <executor/PoolToQueueFlow.hxx>

#include <openlcb/ConfiguredTcpConnection.hxx>
#include <openlcb/DccAccyConsumer.hxx>
#include <openlcb/DccAccyProducer.hxx>
#include <openlcb/MemoryConfig.hxx>
#include <openlcb/SimpleInfoProtocol.hxx>
#include <openlcb/SimpleStack.hxx>
#include <openlcb/TcpDefs.hxx>
#include <openlcb/TractionCvSpace.hxx>
#include <openlcb/TractionProxy.hxx>
#include <openlcb/TractionTrain.hxx>

#include <os/MDNS.hxx>
#include <os/OS.hxx>

#include <utils/AutoSyncFileFlow.hxx>
#include <utils/constants.hxx>
#include <utils/FileUtils.hxx>
#include <utils/format_utils.hxx>
#include <utils/logging.h>
#include <utils/macros.h>
#include <utils/StringPrintf.hxx>
#include <utils/Uninitialized.hxx>

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

#include "sdkconfig.h"

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
using dcc::UpdateLoopBase;

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
#include "HttpStringUtils.h"

#include "DCC/DCCProgrammer.h"
#include "DCC/EStopHandler.h"
#include "DCC/MonitoredHBridge.h"
#include "DCC/RMTTrackDevice.h"
#include "DCC/Turnouts.h"

#include "Interfaces/DCCppProtocol.h"
#include "Interfaces/NextionInterface.h"
#include "Interfaces/WiFiInterface.h"

#include "stateflows/FreeRTOSTaskMonitor.h"
#include "stateflows/StatusDisplay.h"
#include "stateflows/LCCStatCollector.h"
#include "stateflows/StatusLED.h"
#include "stateflows/HC12Radio.h"
#include "stateflows/OTAMonitor.h"

#include "IO/Outputs.h"
#include "IO/Sensors.h"
#include "IO/S88Sensors.h"
#include "IO/RemoteSensors.h"

extern uninitialized<RMTTrackDevice> trackSignal;
extern uninitialized<LocalTrackIf> trackInterface;
extern uninitialized<commandstation::AllTrainNodes> trainNodes;

class CDIHelper
{
public:
/// Creates the XML representation of the configuration structure and saves
/// it to a file on the filesystem. Must be called after SPIFFS.begin() but
/// before calling the {\link create_config_file_if_needed} method. The
/// config file will be re-written whenever there was a change in the
/// contents. It is also necessary to declare the static compiled-in CDI to
/// be empty:
/// ```
///    namespace openlcb {
///    // This will stop openlcb from exporting the CDI memory space
///    // upon start.
///    extern const char CDI_DATA[] = "";
///    }  // namespace openlcb
/// ```
/// @param cfg is the global configuration instance (usually called cfg).
/// @param filename is where the xml file can be stored on the
/// filesystem. For example "/spiffs/cdi.xml".
template <class ConfigDef>
static void create_config_descriptor_xml(
    const ConfigDef &config, const char *filename, bool register_space)
{
  string cdi_string;
  ConfigDef cfg(config.offset());
  cfg.config_renderer().render_cdi(&cdi_string);

  cdi_string += '\0';

  bool need_write = false;
  LOG(INFO, "[CDI] Checking %s...", filename);
  FILE *ff = fopen(filename, "rb");
  if (!ff)
  {
    LOG(INFO, "[CDI] File %s does not exist", filename);
    need_write = true;
  }
  else
  {
    fclose(ff);
    string current_str = read_file_to_string(filename);
    if (current_str != cdi_string)
    {
      LOG(INFO, "[CDI] File %s is not up-to-date", filename);
      need_write = true;
    }
#if LOGLEVEL == VERBOSE
    else
    {
      LOG(INFO, "[CDI] File %s appears up-to-date (len %u vs %u)", filename
        , current_str.size(), cdi_string.size());
    }
#endif
  }
  if (need_write)
  {
    LOG(INFO, "[CDI] Updating %s (len %u)", filename,
        cdi_string.size());
    write_string_to_file(filename, cdi_string);
  }

  if (register_space)
  {
    LOG(INFO, "[CDI] Registering CDI with stack...");
    extern std::unique_ptr<SimpleCanStack> lccStack;
    // Creates list of event IDs for factory reset.
    auto *v = new vector<uint16_t>();
    cfg.handle_events([v](unsigned o) { v->push_back(o); });
    v->push_back(0);
    lccStack->set_event_offsets(v);
    // We leak v because it has to stay alive for the entire lifetime of
    // the stack.

    // Exports the file memory space.
    openlcb::MemorySpace *space = new openlcb::ROFileMemorySpace(filename);
    lccStack->memory_config_handler()->registry()->insert(
        lccStack->node(), openlcb::MemoryConfigDefs::SPACE_CDI, space);
  }
}
};
#if LOCONET_ENABLED
#include <LocoNetESP32UART.h>
extern LocoNetESP32Uart locoNet;
#endif

void initializeLocoNet();

// Returns true if the provided pin is one of the ESP32 pins that has usage
// restrictions. This will always return false if the configuration flag
// ALLOW_USAGE_OF_RESTRICTED_GPIO_PINS is enabled.
bool is_restricted_pin(int8_t);

namespace esp32cs
{
  // C++11 does not have std::make_unique so adding an implementation here to
  // prevent leaking memory when adding a ptr to a container.
  template<typename T, typename... Args>
  static inline std::unique_ptr<T> make_unique(Args&&... args)
  {
      return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
  }
}

/// Utility macro for StateFlow early abort
#define LOG_ESP_ERROR_AND_EXIT_FLOW(name, text, cmd)  \
{                                                     \
  esp_err_t res = cmd;                                \
  if (res != ESP_OK)                                  \
  {                                                   \
    LOG_ERROR("[%s] %s: %s"                           \
            , name, text, esp_err_to_name(res));      \
    return exit();                                    \
  }                                                   \
}

/// Utility macro to initialize a UART as part of a StateFlow
#define CONFIGURE_UART(name, uart, speed, rx, tx, rx_buf, tx_buf) \
{                                                                 \
  LOG(INFO                                                        \
    , "[%s] Initializing UART(%d) at %u baud on RX %d, TX %d"     \
    , name, uart, speed, rx, tx);                                 \
  uart_config_t uart_cfg =                                        \
  {                                                               \
    .baud_rate           = speed,                                 \
    .data_bits           = UART_DATA_8_BITS,                      \
    .parity              = UART_PARITY_DISABLE,                   \
    .stop_bits           = UART_STOP_BITS_1,                      \
    .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,              \
    .rx_flow_ctrl_thresh = 0,                                     \
    .use_ref_tick        = false                                  \
  };                                                              \
  LOG_ESP_ERROR_AND_EXIT_FLOW(name, "uart_param_config",          \
                         uart_param_config(uart, &uart_cfg))      \
  LOG_ESP_ERROR_AND_EXIT_FLOW(name, "uart_set_pin",               \
                         uart_set_pin(uart, tx, rx                \
                                    , UART_PIN_NO_CHANGE          \
                                    , UART_PIN_NO_CHANGE))        \
  LOG_ESP_ERROR_AND_EXIT_FLOW(name, "uart_driver_install",        \
                         uart_driver_install(uart, rx_buf, tx_buf \
                                           , 0, NULL, 0))         \
}

#endif // ESP32_CS_H_