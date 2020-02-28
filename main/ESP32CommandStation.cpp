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

#include "ESP32CommandStation.h"
#include "sdkconfig.h"
#include "CSConfigDescriptor.h"
#include "OTAMonitor.h"

#include <AllTrainNodes.hxx>

#include <dcc/ProgrammingTrackBackend.hxx>
#include <dcc/RailcomHub.hxx>
#include <esp_adc_cal.h>
#include <EStopHandler.h>
#include <executor/PoolToQueueFlow.hxx>
#include <FreeRTOSTaskMonitor.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <openlcb/SimpleInfoProtocol.hxx>
#include <RMTTrackDevice.h>
#include <StatusDisplay.h>
#include <StatusLED.h>
#include <HC12Radio.h>

#include <Sensors.h>
#include <RemoteSensors.h>
#include <Outputs.h>

const char * buildTime = __DATE__ " " __TIME__;

// GCC pre-compiler trick to expand the value from a #define constant
#define OVERRIDE_CONST_EXPAND_VALUE(var, value) OVERRIDE_CONST(var, value)

#if CONFIG_LCC_GC_NEWLINES
///////////////////////////////////////////////////////////////////////////////
// This will generate newlines after GridConnect each packet being sent.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST_TRUE(gc_generate_newlines);
#endif

///////////////////////////////////////////////////////////////////////////////
// Increase the number of memory spaces available at runtime to account for the
// Traction protocol CDI/FDI needs.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST_EXPAND_VALUE(num_memory_spaces, CONFIG_LCC_MEMORY_SPACES);

///////////////////////////////////////////////////////////////////////////////
// Increase the GridConnect buffer size to improve performance by bundling more
// than one GridConnect packet into the same send() call to the socket.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST_EXPAND_VALUE(gridconnect_buffer_size, CONFIG_TCP_MSS);

///////////////////////////////////////////////////////////////////////////////
// This will allow up to 500 usec for the buffer to fill up before sending it
// out over the socket connection.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST_EXPAND_VALUE(gridconnect_buffer_delay_usec
                          , CONFIG_LCC_GC_DELAY_USEC);

///////////////////////////////////////////////////////////////////////////////
// This limites the number of outbound GridConnect packets which limits the
// memory used by the BufferPort.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST_EXPAND_VALUE(gridconnect_bridge_max_outgoing_packets
                          , CONFIG_LCC_GC_OUTBOUND_PACKET_LIMIT);

///////////////////////////////////////////////////////////////////////////////
// This increases number of state flows to invoke before checking for any FDs
// that have pending data.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST_EXPAND_VALUE(executor_select_prescaler
                          , LCC_EXECUTOR_SELECT_PRESCALER);

///////////////////////////////////////////////////////////////////////////////
// This increases the number of local nodes and aliases available for the LCC
// stack. This is needed to allow for virtual train nodes.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST_EXPAND_VALUE(local_nodes_count, CONFIG_LCC_LOCAL_NODE_COUNT);
OVERRIDE_CONST_EXPAND_VALUE(local_alias_cache_size
                          , CONFIG_LCC_LOCAL_NODE_COUNT);

unique_ptr<openlcb::SimpleCanStack> lccStack;

// Esp32ConfigDef comes from CSConfigDescriptor.h and is specific to this
// particular device and target. It defines the layout of the configuration
// memory space and is also used to generate the cdi.xml file. Here we
// instantiate the configuration layout. The argument of offset zero is ignored
// and will be removed later.
static constexpr esp32cs::Esp32ConfigDef cfg(0);

// define the SNIP data for the Command Station.
namespace openlcb
{
  const SimpleNodeStaticValues SNIP_STATIC_DATA =
  {
    4,
    "github.com/atanisoft (Mike Dunston)",
    "ESP32 Command Station",
    "ESP32-v1",
    // TODO: replace with CONFIG_APP_VERSION after v4.1 update
    "1.5.0"
  };
}

// override LCC defaults with the ESP32 CS values.
namespace openlcb
{
  // This will stop openlcb from exporting the CDI memory space upon start.
  const char CDI_DATA[] = "";

  // Path to where OpenMRN should persist general configuration data.
  const char *const CONFIG_FILENAME = LCC_CONFIG_FILE;

  // The size of the memory space to export over the above device.
  const size_t CONFIG_FILE_SIZE = cfg.seg().size() + cfg.seg().offset();

  // Default to store the dynamic SNIP data is stored in the same persistant
  // data file as general configuration data.
  const char *const SNIP_DYNAMIC_FILENAME = LCC_CONFIG_FILE;
}

uninitialized<dcc::LocalTrackIf> trackInterface;

// when the command station starts up the first time the config is blank
// and needs to be reset to factory settings. This class being declared here
// takes care of that.
class FactoryResetHelper : public DefaultConfigUpdateListener {
public:
    UpdateAction apply_configuration(int fd, bool initial_load,
                                     BarrierNotifiable *done) OVERRIDE {
        AutoNotify n(done);
        return UPDATED;
    }

    void factory_reset(int fd) override
    {
        LOG(INFO
          , "[LCC] ESP32 CS factory_reset(%d) invoked, defaulting "
            "configuration", fd);
        cfg.userinfo().name().write(fd, "ESP32 Command Station");
        cfg.userinfo().description().write(fd, "");
    }
};

extern "C" void app_main()
{
  // Setup UART0 115200 8N1 TX: 1, RX: 3, 2k buffer (1k rx, 1k tx)
  uart_config_t uart0 = {
    .baud_rate           = 115200,
    .data_bits           = UART_DATA_8_BITS,         // 8 bit bytes
    .parity              = UART_PARITY_DISABLE,      // no partity
    .stop_bits           = UART_STOP_BITS_1,         // one stop bit
    .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE, // no flow control
    .rx_flow_ctrl_thresh = 0,                        // unused
    .use_ref_tick        = false                     // unused
  };
  uart_param_config(UART_NUM_0, &uart0);
  uart_driver_install(UART_NUM_0, 1024, 1024, 0, NULL, 0);

  const esp_app_desc_t *app_data = esp_ota_get_app_description();

  LOG(INFO, "\n\nESP32 Command Station v%s starting up...", app_data->version);
  LOG(INFO, "Compiled on %s %s using IDF %s", app_data->date, app_data->time
    , app_data->idf_ver);
  LOG(INFO, "Running from: %s", esp_ota_get_running_partition()->label);

  LOG(INFO, "ESP32 Command Station uses the OpenMRN library\n"
            "Copyright (c) 2019-2020, OpenMRN\n"
            "All rights reserved.");

  // Initialize NVS before we do any other initialization as it may be
  // internally used by various components even if we disable it's usage in
  // the WiFi connection stack.
  LOG(INFO, "[NVS] Initializing NVS");
  if (ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_flash_init()) == ESP_ERR_NVS_NO_FREE_PAGES)
  {
    const esp_partition_t* partition =
      esp_partition_find_first(ESP_PARTITION_TYPE_DATA
                             , ESP_PARTITION_SUBTYPE_DATA_NVS
                             , NULL);
    if (partition != NULL)
    {
      LOG(INFO, "[NVS] Erasing partition %s...", partition->label);
      ESP_ERROR_CHECK(esp_partition_erase_range(partition, 0, partition->size));
      ESP_ERROR_CHECK(nvs_flash_init());
    }
  }

  // Configure ADC1 up front to use 12 bit (0-4095) as we use it for all
  // monitored h-bridges.
  LOG(INFO, "[ADC] Configure 12-bit ADC resolution");
  adc1_config_width(ADC_WIDTH_BIT_12);

  // Initialize the Configuration Manager. This will mount SPIFFS and SD
  // (if configured) and then load the CS configuration (if present) or
  // prepare the default configuration. This will also include the LCC
  // Factory reset (if required).
  ConfigurationManager *config = new ConfigurationManager(cfg);

  LOG(INFO, "[LCC] Initializing Stack");
  lccStack.reset(new openlcb::SimpleCanStack(config->getNodeId()));

  // Initialize the enabled modules.
  config->configureWiFi();

  // Initialize the status display module (dependency of WiFi)
  StatusDisplay statusDisplay(lccStack.get(), lccStack->service());

#if CONFIG_NEXTION
  // Initialize the Nextion module (dependency of WiFi)
  LOG(INFO, "[Config] Enabling Nextion module");
  nextionInterfaceInit();
#endif

  init_wifi_endpoints();

  // Initialize the turnout manager and register it with the LCC stack to
  // process accessories packets.
  TurnoutManager turnoutManager(lccStack->node(), lccStack->service());

#if CONFIG_OUTPUTS
  LOG(INFO, "[Config] Enabling GPIO Outputs");
  OutputManager::init();
#endif

#if CONFIG_SENSORS
  LOG(INFO, "[Config] Enabling GPIO Inputs");
  SensorManager::init();
  S88BusManager::init();
  RemoteSensorManager::init();
#endif

#if CONFIG_LOCONET
  LOG(INFO, "[Config] Enabling LocoNet interface");
  initializeLocoNet();
#endif

#if CONFIG_HC12
  esp32cs::HC12Radio hc12(lccStack->service()
#if CONFIG_HC12_UART_UART1
                        , UART_NUM_1
#else
                        , UART_NUM_2
#endif
                        , (gpio_num_t)CONFIG_HC12_RX_PIN
                        , (gpio_num_t)CONFIG_HC12_TX_PIN));
#endif // CONFIG_HC12

  StatusLED statusLED(lccStack->service());

  OTAMonitorFlow ota(lccStack->service());

  // Initialize the factory reset helper for the CS.
  FactoryResetHelper resetHelper;

#if CONFIG_OPS_RAILCOM
  // Initialize the RailCom Hub
  dcc::RailcomHubFlow railComHub(lccStack->service());

#if CONFIG_OPS_RAILCOM_DUMP_PACKETS
  // Add a data dumper for the RailCom Hub
  dcc::RailcomPrintfFlow railComDataDumper(&railComHub);
#endif // CONFIG_OPS_RAILCOM_DUMP_PACKETS

  // Initialize Track Signal Device (both OPS and PROG)
  RMTTrackDevice trackSignal(lccStack.get()
                           , cfg.seg().hbridge().entry(OPS_CDI_TRACK_OUTPUT_INDEX)
                           , cfg.seg().hbridge().entry(PROG_CDI_TRACK_OUTPUT_INDEX)
                           , &railComHub);

#else // NO RAILCOM
  // Initialize Track Signal Device (both OPS and PROG)
  RMTTrackDevice trackSignal(lccStack.get()
                           , cfg.seg().hbridge().entry(OPS_CDI_TRACK_OUTPUT_INDEX)
                           , cfg.seg().hbridge().entry(PROG_CDI_TRACK_OUTPUT_INDEX));
#endif // CONFIG_OPS_RAILCOM

  // Initialize Local Track inteface.
  trackInterface.emplace(lccStack->service()
                       , CONFIG_DCC_PACKET_POOL_SIZE);

  // Open a handle to the track device driver
  int track = ::open("/dev/track", O_RDWR);
  HASSERT(track > 0);
  // pass the track device handle to the track interface
  trackInterface->set_fd(track);

  // Initialize the DCC Update Loop.
  dcc::SimpleUpdateLoop dccUpdateLoop(lccStack->service()
                                    , &trackInterface.value());

  // Attach the DCC update loop to the track interface
  PoolToQueueFlow<Buffer<dcc::Packet>> dccPacketFlow(lccStack->service()
                                                   , trackInterface->pool()
                                                   , &dccUpdateLoop);

  // Initialize the e-stop event handler
  esp32cs::EStopHandler eStop(lccStack->node());

  // Initialize the Programming Track backend handler
  ProgrammingTrackBackend
    progTrackBackend(lccStack->service()
                   , std::bind(&RMTTrackDevice::enable_prog_output
                             , &trackSignal)
                   , std::bind(&RMTTrackDevice::disable_prog_output
                             , &trackSignal));

  // Initialize the OpenMRN stack, this needs to be done *AFTER* all other LCC
  // dependent components as it will initiate configuration load and factory
  // reset calls.
  config->configureLCC();

  // Initialize the DCC++ protocol adapter
  DCCPPProtocolHandler::init();

  // Initialize the Traction Protocol support
  openlcb::TrainService trainService(lccStack->iface());

  // Initialize the train database
  esp32cs::Esp32TrainDatabase trainDb(lccStack.get());

  // Initialize the Train Search and Train Manager.
  commandstation::AllTrainNodes trainNodes(&trainDb
                                         , &trainService
                                         , lccStack->info_flow()
                                         , lccStack->memory_config_handler()
                                         , trainDb.get_readonly_train_cdi()
                                         , trainDb.get_readonly_temp_train_cdi());


  // Task Monitor, periodically dumps runtime state to STDOUT.
  LOG(VERBOSE, "Starting FreeRTOS Task Monitor");
  FreeRTOSTaskMonitor taskMon(lccStack->service());

  LOG(INFO, "\n\nESP32 Command Station Startup complete!\n");
  Singleton<StatusDisplay>::instance()->status("ESP32-CS Started");

  // donate our task thread to OpenMRN executor.
  lccStack->loop_executor();
}

// TODO: move these back to DCCppProtocol after moving sensors and outputs to
// components tree

DCC_PROTOCOL_COMMAND_HANDLER(ConfigErase,
[](const vector<string> arguments)
{
  Singleton<TurnoutManager>::instance()->clear();
#if CONFIG_ENABLE_SENSORS
  SensorManager::clear();
  SensorManager::store();
#if CONFIG_S88
  S88BusManager::clear();
  S88BusManager::store();
#endif
#endif
#if CONFIG_ENABLE_OUTPUTS
  OutputManager::clear();
  OutputManager::store();
#endif
  return COMMAND_SUCCESSFUL_RESPONSE;
})

DCC_PROTOCOL_COMMAND_HANDLER(ConfigStore,
[](const vector<string> arguments)
{
  return StringPrintf("<e %d %d %d>"
                    , Singleton<TurnoutManager>::instance()->getTurnoutCount()
#if CONFIG_ENABLE_SENSORS
                    , SensorManager::store()
#if CONFIG_S88
                    + S88BusManager::store()
#endif
#else
                    , 0
#endif
#if CONFIG_ENABLE_OUTPUTS
                    , OutputManager::store()
#else
                    , 0
#endif
    );
})

DCC_PROTOCOL_COMMAND_HANDLER(StatusCommand,
[](const vector<string> arguments)
{
  wifi_mode_t mode;
  const esp_app_desc_t *app_data = esp_ota_get_app_description();
  string status = StringPrintf("<iDCC++ ESP32 Command Station: V-%s / %s %s>"
                , app_data->version, app_data->date, app_data->time);
  status += Singleton<RMTTrackDevice>::instance()->get_state_for_dccpp();
  auto trains = Singleton<commandstation::AllTrainNodes>::instance();
  for (size_t id = 0; id < trains->size(); id++)
  {
    auto nodeid = trains->get_train_node_id(id);
    if (nodeid)
    {
      auto impl = trains->get_train_impl(nodeid);
      status += convert_loco_to_dccpp_state(impl, id);
    }
  }
  status += Singleton<TurnoutManager>::instance()->get_state_for_dccpp();
#if CONFIG_ENABLE_OUTPUTS
  status += OutputManager::get_state_for_dccpp();
#endif
  if (esp_wifi_get_mode(&mode) == ESP_OK)
  {
    tcpip_adapter_ip_info_t ip_info;
    if (mode == WIFI_MODE_STA || mode == WIFI_MODE_APSTA)
    {
      if (tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info) == ESP_OK)
      {
        status += StringPrintf("<N1: " IPSTR ">", IP2STR(&ip_info.ip));
      }
      if (tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_AP, &ip_info) == ESP_OK)
      {
        status += StringPrintf("<N1: " IPSTR ">", IP2STR(&ip_info.ip));
      }
    }
    else if (mode != WIFI_MODE_NULL &&
             tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_AP, &ip_info) == ESP_OK)
    {
      status += StringPrintf("<N1: " IPSTR ">", IP2STR(&ip_info.ip));
    }
  }
  return status;
})