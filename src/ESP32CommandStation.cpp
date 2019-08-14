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
#include "cdi/CSConfigDescriptor.h"

const char * buildTime = __DATE__ " " __TIME__;

// Allow usage of ::select() for GridConnect TCP connections.
OVERRIDE_CONST_TRUE(gridconnect_tcp_use_select);

// Increased GridConnect buffer size (improves performance).
OVERRIDE_CONST(gridconnect_buffer_size, 3512);

// Increased delay in flushing the GridConnect TCP data.
OVERRIDE_CONST(gridconnect_buffer_delay_usec, 1000);

// Generate newlines after GridConnect packets on TCP.
//OVERRIDE_CONST_TRUE(gc_generate_newlines);

// Increased number of state flows to invoke before checking for ::select
// timeouts
OVERRIDE_CONST(executor_select_prescaler, 30);

// Increased number of outbound GridConnect packets to queue.
//OVERRIDE_CONST(gridconnect_bridge_max_outgoing_packets, 5);

// Increased number of local nodes to account for TrainNode proxy nodes
OVERRIDE_CONST(local_nodes_count, 30);
OVERRIDE_CONST(local_alias_cache_size, 30);

// Uncomment to have all railcom data printed as it is received.
//OVERRIDE_CONST_TRUE("enable_railcom_packet_dump");

// Uncomment to list task statistics periodically.
//OVERRIDE_CONST_TRUE(cs_task_list_report);

// Uncomment to force a factory reset of all configuration data on startup.
// WARNING: THIS WILL CLEAR *ALL* PERSISTENT DATA!
// OVERRIDE_CONST_TRUE(cs_force_factory_reset);

std::unique_ptr<OpenMRN> openmrn;
// note the dummy string below is required due to a bug in the GCC compiler
// for the ESP32
string dummystring("abcdef");

// Esp32ConfigDef comes from CSConfigDescriptor.h and is specific to this
// particular device and target. It defines the layout of the configuration
// memory space and is also used to generate the cdi.xml file. Here we
// instantiate the configuration layout. The argument of offset zero is ignored
// and will be removed later.
static constexpr esp32cs::Esp32ConfigDef cfg(0);

// define the SNIP data for the Command Station.
namespace openlcb {
  const SimpleNodeStaticValues SNIP_STATIC_DATA = {
    4,
    "github.com/atanisoft (Mike Dunston)",
    "ESP32 Command Station",
    "ESP32-v1",
    ESP32CS_VERSION
  };
}

// override LCC defaults with the ESP32 CS values.
namespace openlcb
{
  // This will stop openlcb from exporting the CDI memory space upon start.
  const char CDI_DATA[] = "";

  // Path to where OpenMRN should persist general configuration data.
  const char *const CONFIG_FILENAME = LCC_NODE_CONFIG_FILE;

  // The size of the memory space to export over the above device.
  const size_t CONFIG_FILE_SIZE = cfg.seg().size() + cfg.seg().offset();

  // Default to store the dynamic SNIP data is stored in the same persistant
  // data file as general configuration data.
  const char *const SNIP_DYNAMIC_FILENAME = CONFIG_FILENAME;
}

unique_ptr<RMTTrackDevice> trackSignal;
unique_ptr<LocalTrackIf> trackInterface;
unique_ptr<RailcomHubFlow> railComHub;

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
        LOG(VERBOSE, "Factory Reset Helper invoked");
        cfg.userinfo().name().write(fd, "ESP32 Command Station");
        cfg.userinfo().description().write(fd, "");
    }
};

void openmrn_loop_task(void *unused)
{
  while(true) {
    openmrn->loop();
    // yield to other tasks by going to sleep for 1ms
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

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

  LOG(INFO, "\n\nESP32 Command Station v%s starting up...", ESP32CS_VERSION);
  LOG(INFO, "ESP32 Command Station uses the OpenMRN library\n"
            "Copyright (c) 2019, OpenMRN\nAll rights reserved.");

  // Initialize the Arduino-ESP32 stack early in the startup flow.
  LOG(INFO, "[Arduino] Initializing Arduino stack");
  initArduino();

  // Configure ADC1 up front to use 12 bit (0-4095) as we use it for all
  // monitored h-bridges.
  LOG(INFO, "[ADC] Configure 12-bit ADC resolution");
  adc1_config_width(ADC_WIDTH_BIT_12);

  // Initialize the Configuration Manager. This will mount SPIFFS and SD
  // (if configured) and then load the CS configuration (if present) or
  // prepare the default configuration.
  configStore.reset(new ConfigurationManager());

  // Pre-create LCC configuration directory.
  mkdir(LCC_PERSISTENT_CONFIG_DIR, ACCESSPERMS);

  bool factoryResetNeeded = config_cs_force_factory_reset() == CONSTANT_TRUE ||
                            config_lcc_force_factory_reset() == CONSTANT_TRUE;

  struct stat statbuf;
  // check the LCC config file to ensure it is the expected size. If not
  // force a factory reset.
  if (stat(openlcb::CONFIG_FILENAME, &statbuf) >= 0 &&
      statbuf.st_size < openlcb::CONFIG_FILE_SIZE)
  {
    LOG(WARNING
      , "[LCC] Corrupt configuration file detected, %s is too small: %ld bytes"
        ", expected: %d bytes"
      , openlcb::CONFIG_FILENAME, statbuf.st_size, openlcb::CONFIG_FILE_SIZE);
    factoryResetNeeded = true;
  }

  if (factoryResetNeeded)
  {
    LOG(WARNING, "[LCC] Forcing factory reset!");
    unlink(openlcb::CONFIG_FILENAME);
    unlink(LCC_NODE_CDI_FILE);
  }

  // Initialize the OpenMRN stack.
  openmrn.reset(new OpenMRN(configStore->getNodeId()));

  // Initialize the enabled modules.
  configStore->configureEnabledModules(openmrn->stack());

  // Initialize the factory reset helper for the CS.
  FactoryResetHelper resetHelper;

  // Initialize the OpenMRN stack (CAN and WiFi interfaces).
  configStore->configureLCC(openmrn.get(), cfg);

  // Initialize the RailCom Hub
  railComHub.reset(new RailcomHubFlow(openmrn->stack()->service()));

  // Initialize Track Signal Device (both OPS and PROG)
  trackSignal.reset(new RMTTrackDevice(openmrn->stack()
                                     , railComHub.get()
                                     , cfg.seg().hbridge().entry(0)
                                     , cfg.seg().hbridge().entry(1)));

  // Initialize Local Track inteface.
  trackInterface.reset(new LocalTrackIf(openmrn->stack()->service()
                                      , config_cs_track_pool_size()));

  // Initialize the MemorySpace handler for CV read/write.
  TractionCvSpace cvMemorySpace(openmrn->stack()->memory_config_handler()
                              , trackInterface.get()
                              , railComHub.get()
                              , MemoryConfigDefs::SPACE_DCC_CV);

  // Open a handle to the track device driver
  int track = ::open("/dev/track", O_RDWR);
  HASSERT(track > 0);
  // pass the track device handle to the track interface
  trackInterface->set_fd(track);

  // Initialize the DCC Update Loop.
  SimpleUpdateLoop dccUpdateLoop(openmrn->stack()->service()
                               , trackInterface.get());

  // Attach the DCC update loop to the track interface
  PoolToQueueFlow<Buffer<Packet>> dccPacketFlow(openmrn->stack()->service()
                                              , trackInterface->pool()
                                              , &dccUpdateLoop);

  // Add a data dumper for the RailCom Hub
  unique_ptr<RailcomPrintfFlow> railComDataDumper;
  if (config_enable_railcom_packet_dump() == CONSTANT_TRUE)
  {
    railComDataDumper.reset(new RailcomPrintfFlow(railComHub.get()));
  }

  // Initialize the Programming Track backend handler
  ProgrammingTrackBackend
    progTrackBackend(openmrn->stack()->service()
                   , std::bind(&RMTTrackDevice::enable_prog_output
                             , trackSignal.get())
                   , std::bind(&RMTTrackDevice::disable_prog_output
                             , trackSignal.get()));

  // Initialize the DCC++ protocol adapter
  DCCPPProtocolHandler::init();

  wifiInterface.init();

  nextionInterfaceInit();

  // Initialize the turnout manager and register it with the LCC stack to
  // process accessories packets.
  turnoutManager.reset(new TurnoutManager(openmrn->stack()->node()));

  // Initialize the Traction Protocol support
  TrainService trainService(openmrn->stack()->iface());

  // Initialize the locomotive manager
  locoManager.reset(new LocomotiveManager(openmrn->stack()->node(), &trainService));

  esp32cs::Esp32TrainDatabase trainDb;

  // Start the OpenMRN stack.
  openmrn->begin();

/*
  commandstation::AllTrainNodes
    allTrainNodes(&trainDb, &trainService
                , openmrn->stack()->info_flow()
                , openmrn->stack()->memory_config_handler()
                , trainDb.get_readonly_train_cdi()
                , trainDb.get_readonly_temp_train_cdi());
*/

  openmrn->stack()->executor()->add(new CallbackExecutable([]()
  {
    LOG(INFO, "[OpenMRN] Starting loop task on core:%d", APP_CPU_NUM);
    xTaskCreatePinnedToCore(openmrn_loop_task, "OpenMRN"
                          , openmrn_arduino::OPENMRN_STACK_SIZE, nullptr, 1
                          , nullptr, APP_CPU_NUM);
  }));

  LOG(INFO, "\n\nESP32 Command Station Startup complete!\n");
  Singleton<InfoScreen>::instance()->replaceLine(
    INFO_SCREEN_ROTATING_STATUS_LINE, "ESP32-CS Started");

  // donate our task thread to OpenMRN executor.
  openmrn->loop_executor();
}

bool is_restricted_pin(int8_t pin)
{
  vector<uint8_t> restrictedPins
  {
#if !ALLOW_USAGE_OF_RESTRICTED_GPIO_PINS
    0,                        // Bootstrap / Firmware Flash Download
    1,                        // UART0 TX
    2,                        // Bootstrap / Firmware Flash Download
    3,                        // UART0 RX
    5,                        // Bootstrap
    6, 7, 8, 9, 10, 11,       // on-chip flash pins
    12, 15,                   // Bootstrap / SD pins
#endif
    OPS_ENABLE_PIN            // OPS h-bridge enable
  , OPS_SIGNAL_PIN            // OPS signal
  , PROG_ENABLE_PIN           // PROG h-bridge enable
  , PROG_SIGNAL_PIN           // PROG signal
#if RAILCOM_BRAKE_ENABLE_PIN != NOT_A_PIN
  , RAILCOM_BRAKE_ENABLE_PIN  // RailCom brake
#endif
#if RAILCOM_ENABLE_PIN != NOT_A_PIN
  , RAILCOM_ENABLE_PIN        // RailCom enable
#endif
#if RAILCOM_SHORT_PIN != NOT_A_PIN
  , RAILCOM_SHORT_PIN         // RailCom short detection
#endif
#if RAILCOM_UART_RX_PIN != NOT_A_PIN
  , RAILCOM_UART_RX_PIN       // RailCom UART RX
#endif
#if LCC_CAN_RX_PIN != NOT_A_PIN
  , LCC_CAN_RX_PIN            // LCC CAN RX
#endif
#if LCC_CAN_TX_PIN != NOT_A_PIN
  , LCC_CAN_TX_PIN            // LCC CAN TX
#endif
#if STATUS_LED_ENABLED
  , STATUS_LED_DATA_PIN       // LED Data Pin
#endif
#if HC12_RADIO_ENABLED
  , HC12_RX_PIN               // HC12 RX
  , HC12_TX_PIN               // HC12 TX
#endif
#if NEXTION_ENABLED
  , NEXTION_UART_RX_PIN       // Nextion RX
  , NEXTION_UART_TX_PIN       // Nextion TX
#endif
#if INFO_SCREEN_ENABLED
  , INFO_SCREEN_SDA_PIN       // SDA
  , INFO_SCREEN_SCL_PIN       // SCL
#ifdef INFO_SCREEN_RESET_PIN
  , INFO_SCREEN_RESET_PIN     // Reset pin
#endif
#endif
#if LOCONET_ENABLED
  , LOCONET_RX_PIN            // LocoNet RX
  , LOCONET_TX_PIN            // LocoNet TX
#endif
#if S88_ENABLED
  , S88_CLOCK_PIN             // S88 Clock
  , S88_RESET_PIN             // S88 Reset
  , S88_LOAD_PIN              // S88 Load
#endif
  };
  return std::find(restrictedPins.begin()
                 , restrictedPins.end()
                 , pin) != restrictedPins.end();
}