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

#include "sdkconfig.h"
#include "CSConfigDescriptor.h"
#include "ESP32TrainDatabase.h"
#include "OTAMonitor.h"

#include <AllTrainNodes.hxx>
#include <ConfigurationManager.h>
#include <dcc/ProgrammingTrackBackend.hxx>
#include <dcc/RailcomHub.hxx>
#include <dcc/SimpleUpdateLoop.hxx>
#include <DCCSignalVFS.h>
#include <driver/uart.h>
#include <DuplexedTrackIf.h>
#include <esp_adc_cal.h>
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <executor/PoolToQueueFlow.hxx>
#include <FreeRTOSTaskMonitor.h>
#include <HC12Radio.h>
#include <Httpd.h>
#include <HttpStringUtils.h>
#include <LCCStackManager.h>
#include <LCCWiFiManager.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <openlcb/SimpleInfoProtocol.hxx>
#include <os/MDNS.hxx>
#include <StatusDisplay.h>
#include <StatusLED.h>
#include <Turnouts.h>

#if CONFIG_GPIO_SENSORS
#include <Sensors.h>
#include <RemoteSensors.h>
#if CONFIG_GPIO_S88
#include <S88Sensors.h>
#endif // CONFIG_GPIO_S88
#endif // CONFIG_GPIO_SENSORS
#if CONFIG_GPIO_OUTPUTS
#include <Outputs.h>
#endif // CONFIG_GPIO_OUTPUTS

#if CONFIG_JMRI
#include <JmriInterface.h>
#endif

const char * buildTime = __DATE__ " " __TIME__;

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
OVERRIDE_CONST_DEFERRED(num_memory_spaces, CONFIG_LCC_MEMORY_SPACES);

///////////////////////////////////////////////////////////////////////////////
// Increase the GridConnect buffer size to improve performance by bundling more
// than one GridConnect packet into the same send() call to the socket.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST_DEFERRED(gridconnect_buffer_size, (CONFIG_LWIP_TCP_MSS / 2));

///////////////////////////////////////////////////////////////////////////////
// Increase the time for the buffer to fill up before sending it out over the
// socket connection.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST_DEFERRED(gridconnect_buffer_delay_usec
                      , CONFIG_LCC_GC_DELAY_USEC);

#if CONFIG_LCC_USE_SELECT
///////////////////////////////////////////////////////////////////////////////
// Enable usage of select() for GridConnect connections.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST_TRUE(gridconnect_tcp_use_select);
#endif // CONFIG_LCC_USE_SELECT

///////////////////////////////////////////////////////////////////////////////
// This limites the number of outbound GridConnect packets which limits the
// memory used by the BufferPort.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST_DEFERRED(gridconnect_bridge_max_outgoing_packets
                      , CONFIG_LCC_GC_OUTBOUND_PACKET_LIMIT);

///////////////////////////////////////////////////////////////////////////////
// This increases number of state flows to invoke before checking for any FDs
// that have pending data.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST_DEFERRED(executor_select_prescaler
                      , CONFIG_LCC_EXECUTOR_SELECT_PRESCALER);

///////////////////////////////////////////////////////////////////////////////
// This increases the number of local nodes and aliases available for the LCC
// stack. This is needed to allow for virtual train nodes.
///////////////////////////////////////////////////////////////////////////////
OVERRIDE_CONST_DEFERRED(local_nodes_count, CONFIG_LCC_LOCAL_NODE_COUNT);
OVERRIDE_CONST_DEFERRED(local_alias_cache_size, CONFIG_LCC_LOCAL_NODE_COUNT);

std::unique_ptr<openlcb::SimpleStackBase> lccStack;

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
    CONFIG_ESP32CS_HW_VERSION,
    CONFIG_ESP32CS_SW_VERSION
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

// when the command station starts up the first time the config is blank
// and needs to be reset to factory settings. This class being declared here
// takes care of that.
class FactoryResetHelper : public DefaultConfigUpdateListener
{
public:
  UpdateAction apply_configuration(int fd, bool initial_load,
                                    BarrierNotifiable *done) override
  {
    AutoNotify n(done);
    return UPDATED;
  }

  void factory_reset(int fd) override
  {
    LOG(INFO, "[LCC] ESP32 Command Station factory_reset(%d) triggered.", fd);
    cfg.userinfo().name().write(fd, "ESP32 Command Station");
    cfg.userinfo().description().write(fd, "");
  }
};

void init_webserver(const esp32cs::Esp32ConfigDef &cfg);

extern "C" void app_main()
{
  esp_log_level_set("*", ESP_LOG_ERROR);

  // Setup UART0 115200 8N1 TX: 1, RX: 3, 2k buffer (1k rx, 1k tx)
  uart_config_t uart0 =
  {
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

  LOG(INFO, "\n\nESP32 Command Station v%s (%s) starting up...", CONFIG_ESP32CS_SW_VERSION, app_data->version);
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
  ConfigurationManager config(cfg);

  esp32cs::LCCStackManager stackManager(cfg);

  esp32cs::LCCWiFiManager wifiManager(stackManager.stack(), cfg);
  
  // Initialize the Http server and mDNS instance
  MDNS mDNS;
  http::Httpd httpd(&mDNS);

  // Initialize the status display module (dependency of WiFi)
  StatusDisplay statusDisplay(stackManager.stack()
                            , stackManager.service());

#if CONFIG_NEXTION
  // Initialize the Nextion module (dependency of WiFi)
  LOG(INFO, "[Config] Enabling Nextion module");
  nextionInterfaceInit(stackManager.service());
#endif // CONFIG_NEXTION

  init_webserver(cfg);

#if CONFIG_JMRI
  init_jmri_interface();
#endif // CONFIG_JMRI

  // Initialize the turnout manager and register it with the LCC stack to
  // process accessories packets.
  TurnoutManager turnoutManager(stackManager.node()
                              , stackManager.service());

#if CONFIG_GPIO_OUTPUTS
  LOG(INFO, "[Config] Enabling GPIO Outputs");
  OutputManager::init();
#endif // CONFIG_GPIO_OUTPUTS

#if CONFIG_GPIO_SENSORS
  LOG(INFO, "[Config] Enabling GPIO Inputs");
  SensorManager::init();
  RemoteSensorManager::init();
#if CONFIG_GPIO_S88
  S88BusManager s88(stackManager.node());
#endif // CONFIG_GPIO_S88
#endif // CONFIG_GPIO_SENSORS

#if CONFIG_LOCONET
  LOG(INFO, "[Config] Enabling LocoNet interface");
  initializeLocoNet();
#endif // CONFIG_LOCONET

#if CONFIG_HC12
  esp32cs::HC12Radio hc12(stackManager.service()
                        , (uart_port_t)CONFIG_HC12_UART
                        , (gpio_num_t)CONFIG_HC12_RX_PIN
                        , (gpio_num_t)CONFIG_HC12_TX_PIN));
#endif // CONFIG_HC12

#if CONFIG_STATUS_LED
  StatusLED statusLED(stackManager.service());
#endif // CONFIG_STATUS_LED

  // cppcheck-suppress UnusedVar
  OTAMonitorFlow ota(stackManager.service());

  // Initialize the factory reset helper for the CS.
  FactoryResetHelper resetHelper;

  // Initialize the DCC VFS adapter, this will also initialize the DCC signal
  // generation code.
  esp32cs::init_dcc_vfs(stackManager.node(), stackManager.service()
                      , cfg.seg().hbridge().entry(esp32cs::OPS_CDI_TRACK_OUTPUT_IDX)
                      , cfg.seg().hbridge().entry(esp32cs::PROG_CDI_TRACK_OUTPUT_IDX));

  int ops_track = ::open(
    StringPrintf("/dev/track/%s", CONFIG_OPS_TRACK_NAME).c_str(), O_WRONLY);
  HASSERT(ops_track > 0);

  int prog_track = ::open(
    StringPrintf("/dev/track/%s", CONFIG_PROG_TRACK_NAME).c_str(), O_WRONLY);
  HASSERT(prog_track > 0);

  // Initialize Local Track inteface.
  esp32cs::DuplexedTrackIf track(stackManager.service()
                               , CONFIG_DCC_PACKET_POOL_SIZE
                               , ops_track, prog_track);

  // Initialize the DCC Update Loop.
  dcc::SimpleUpdateLoop dccUpdateLoop(stackManager.service(), &track);

  // Attach the DCC update loop to the track interface
  PoolToQueueFlow<Buffer<dcc::Packet>> dccPacketFlow(stackManager.service()
                                                   , track.pool()
                                                   , &dccUpdateLoop);

  // Starts the OpenMRN stack, this needs to be done *AFTER* all other LCC
  // dependent components as it will initiate configuration load and factory
  // reset calls.
  stackManager.start(config.is_sd());

  // Initialize the DCC++ protocol adapter
  DCCPPProtocolHandler::init();

  // Initialize the Traction Protocol support
  openlcb::TrainService trainService(stackManager.stack()->iface());

  // Initialize the train database
  esp32cs::Esp32TrainDatabase trainDb(stackManager.stack());

  // Initialize the Train Search and Train Manager.
  commandstation::AllTrainNodes trainNodes(&trainDb
                                         , &trainService
                                         , stackManager.info_flow()
                                         , stackManager.memory_config_handler()
                                         , trainDb.get_train_cdi()
                                         , trainDb.get_temp_train_cdi());

  // Task Monitor, periodically dumps runtime state to STDOUT.
  LOG(VERBOSE, "Starting FreeRTOS Task Monitor");
  FreeRTOSTaskMonitor taskMon(stackManager.service());

  LOG(INFO, "\n\nESP32 Command Station Startup complete!\n");
  Singleton<StatusDisplay>::instance()->status("ESP32-CS Started");

  // donate our task thread to OpenMRN executor.
  stackManager.stack()->loop_executor();
}
