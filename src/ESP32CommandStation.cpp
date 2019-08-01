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
#include "stateflows/FreeRTOSTaskMonitor.h"

#include "cdi/CSConfigDescriptor.h"

using openlcb::ConfigDef;

const char * buildTime = __DATE__ " " __TIME__;

#ifndef ALLOW_USAGE_OF_RESTRICTED_GPIO_PINS
vector<uint8_t> restrictedPins
{
  0, // Bootstrap / Firmware Flash Download
  1, // UART0 TX
  2, // Bootstrap / Firmware Flash Download
  3, // UART0 RX
  5, // Bootstrap
  6, 7, 8, 9, 10, 11, // on-chip flash pins
  12, 15 // Bootstrap / SD pins
};
#else
vector<uint8_t> restrictedPins;
#endif

std::unique_ptr<OpenMRN> openmrn;
// note the dummy string below is required due to a bug in the GCC compiler
// for the ESP32
string dummystring("abcdef");

// ConfigDef comes from CSConfigDescriptor.h and is specific to this particular
// device and target. It defines the layout of the configuration memory space
// and is also used to generate the cdi.xml file. Here we instantiate the
// configuration layout. The argument of offset zero is ignored and will be
// removed later.
static constexpr ConfigDef cfg(0);

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

OVERRIDE_CONST_TRUE(gridconnect_tcp_use_select);
OVERRIDE_CONST(gridconnect_buffer_size, 3512);
OVERRIDE_CONST(gridconnect_buffer_delay_usec, 1000);
OVERRIDE_CONST_TRUE(gc_generate_newlines);
OVERRIDE_CONST(executor_select_prescaler, 60);
//OVERRIDE_CONST(gridconnect_bridge_max_outgoing_packets, );

// increase the local node count so that we can have a few train nodes active
OVERRIDE_CONST(local_nodes_count, 30);
OVERRIDE_CONST(local_alias_cache_size, 30);

// state flows that are not defined elsewhere
unique_ptr<FreeRTOSTaskMonitor> taskMonitor;
unique_ptr<LCCStatCollector> lccStatCollector;
unique_ptr<OTAMonitorFlow> otaMonitor;

unique_ptr<RMTTrackDevice> trackSignal;
unique_ptr<LocalTrackIf> trackInterface;
unique_ptr<SimpleUpdateLoop> dccUpdateLoop;
unique_ptr<PoolToQueueFlow<Buffer<dcc::Packet>>> dccPacketFlow;
unique_ptr<RailcomHubFlow> railComHub;
unique_ptr<RailcomPrintfFlow> railComDataDumper;
unique_ptr<ProgrammingTrackBackend> progTrackBackend;
unique_ptr<DccAccyConsumer> accessoryConsumer;
unique_ptr<TractionCvSpace> cvMemorySpace;

#if CONFIG_USE_SD
#define CDI_CONFIG_PREFIX "/sdcard"
#else
#define CDI_CONFIG_PREFIX "/spiffs"
#endif

namespace openlcb
{
    // Name of CDI.xml to generate dynamically.
    const char CDI_FILENAME[] = CDI_CONFIG_PREFIX LCC_CDI_FILE;

    // This will stop openlcb from exporting the CDI memory space upon start.
    const char CDI_DATA[] = "";

    // Path to where OpenMRN should persist general configuration data.
    const char *const CONFIG_FILENAME = CDI_CONFIG_PREFIX LCC_CONFIG_FILE;

    // The size of the memory space to export over the above device.
    const size_t CONFIG_FILE_SIZE = cfg.seg().size() + cfg.seg().offset();

    // Default to store the dynamic SNIP data is stored in the same persistant
    // data file as general configuration data.
    const char *const SNIP_DYNAMIC_FILENAME = CONFIG_FILENAME;

    const char *const CONFIG_DIR = CDI_CONFIG_PREFIX LCC_CONFIG_DIR;
}

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

std::unique_ptr<FactoryResetHelper> resetHelper;

bool otaComplete = false;
esp_ota_handle_t otaInProgress = 0;

#if CPULOAD_REPORTING
#include <freertos_drivers/arduino/CpuLoad.hxx>

#include <esp_spi_flash.h>
#include <esp32-hal-timer.h>
CpuLoad cpuLogTracker;
hw_timer_t *cpuTickTimer = nullptr;
CpuLoadLog cpuLoadLogger(openmrn.stack()->service());

constexpr uint8_t CPULOAD_TIMER_NUMBER = 3;
constexpr uint8_t CPULOAD_TIMER_DIVIDER = 80;

os_thread_t cpuTickTaskHandle;

void *cpuTickTask(void *param) {
    while(true) {
        // go to sleep until next interval
        ulTaskNotifyTake(true, portMAX_DELAY);
        // Retrieves the vtable pointer from the currently running executable.
        unsigned *pp = (unsigned *)openmrn->stack()->executor()->current();
        cpuload_tick(pp ? pp[0] | 1 : 0);
    }
    return nullptr;
}

void IRAM_ATTR cpuTickTimerCallback() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(cpuTickTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR();
}
#endif // CPULOAD_REPORTING

extern "C" {

/// Reboots the ESP32 via the arduino-esp32 provided restart function.
void reboot()
{
  // shutdown and cleanup the configuration manager
  configStore.reset(nullptr);

  LOG(INFO, "Restarting ESP32 Command Station");
  // restart the node
  esp_restart();
}

ssize_t os_get_free_heap()
{
  return heap_caps_get_free_size(MALLOC_CAP_8BIT);
}

}

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

  LOG(INFO, "\n\nESP32 Command Station v%s starting up", ESP32CS_VERSION);

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
  mkdir(openlcb::CONFIG_DIR, ACCESSPERMS);

#if LCC_FORCE_FACTORY_RESET_ON_STARTUP
  LOG(WARNING, "[LCC] Forcing factory reset");
  unlink(openlcb::CDI_FILENAME);
  unlink(openlcb::CONFIG_FILENAME);
#endif

  // Initialize the OpenMRN stack.
  openmrn.reset(new OpenMRN(configStore->getNodeId()));

  // Initialize global state flows.
  hc12.reset(new HC12Radio(openmrn->stack()));
  infoScreen.reset(new InfoScreen(openmrn->stack()));
  lccStatCollector.reset(new LCCStatCollector(openmrn->stack()));
  otaMonitor.reset(new OTAMonitorFlow(openmrn->stack()));
  statusLED.reset(new StatusLED(openmrn->stack()));
  taskMonitor.reset(new FreeRTOSTaskMonitor(openmrn->stack()));

  // Initialize the factory reset helper for the CS.
  resetHelper.reset(new FactoryResetHelper());

  // Initialize the WiFi Manager.
  configStore->configureWiFi(openmrn->stack(), cfg.seg().wifi());

  // Initialize the CAN interface.
  configStore->configureCAN(openmrn.get());

  // Create the CDI.xml dynamically if it doesn't already exist.
  openmrn->create_config_descriptor_xml(cfg, openlcb::CDI_FILENAME);

  // Create the default internal configuration file if it doesn't already exist.
  openmrn->stack()->create_config_file_if_needed(cfg.seg().internal_config()
                                               , ESP32CS_NUMERIC_VERSION
                                               , openlcb::CONFIG_FILE_SIZE);

  // Initialize the RailCom Hub
  railComHub.reset(new RailcomHubFlow(openmrn->stack()->service()));

  // Initialize Track Signal Device (both OPS and PROG)
  trackSignal.reset(new RMTTrackDevice(openmrn->stack()
                                     , railComHub.get()
                                     , cfg.seg().hbridge().entry(0)
                                     , cfg.seg().hbridge().entry(1)));

  // Initialize Local Track inteface
  trackInterface.reset(new LocalTrackIf(openmrn->stack()->service(), 10));

  // Initialize DCC Accessory consumer
  accessoryConsumer.reset(new DccAccyConsumer(openmrn->stack()->node()
                                            , trackInterface.get()));

  cvMemorySpace.reset(new TractionCvSpace(openmrn->stack()->memory_config_handler()
                                        , trackInterface.get()
                                        , railComHub.get()
                                        , MemoryConfigDefs::SPACE_DCC_CV));

  // Open a handle to the track device driver
  int track = ::open("/dev/track", O_RDWR);
  HASSERT(track > 0);
  // pass the track device handle to the track interface
  trackInterface->set_fd(track);

  // Initialize the DCC Update Loop.
  dccUpdateLoop.reset(
    new SimpleUpdateLoop(openmrn->stack()->service(), trackInterface.get()));

  // Attach the DCC update loop to the track interface
  dccPacketFlow.reset(
    new PoolToQueueFlow<Buffer<dcc::Packet>>(openmrn->stack()->service()
                                           , trackInterface->pool()
                                           , dccUpdateLoop.get()));

  // Add a data dumper for the RailCom Hub
  railComDataDumper.reset(new RailcomPrintfFlow(railComHub.get()));

  // Initialize the Programming Track backend handler
  progTrackBackend.reset(
    new ProgrammingTrackBackend(openmrn->stack()->service()
                              , std::bind(&RMTTrackDevice::enable_prog_output
                                        , trackSignal.get())
                              , std::bind(&RMTTrackDevice::disable_prog_output
                                        , trackSignal.get())));

  // Initialize the DCC++ protocol adapter
  DCCPPProtocolHandler::init();

  wifiInterface.init();

  nextionInterfaceInit();

  LocomotiveManager::init(openmrn->stack()->node());

  // Initialize the turnout manager and register it with the LCC stack to
  // process accessories packets.
  turnoutManager.reset(new TurnoutManager(openmrn->stack()->node()));

  // Start the OpenMRN stack.
  openmrn->begin();

#if CPULOAD_REPORTING
  os_thread_create(&cpuTickTaskHandle, "loadtick", 1, 0, &cpuTickTask, nullptr);
  cpuTickTimer = timerBegin(CPULOAD_TIMER_NUMBER, CPULOAD_TIMER_DIVIDER, true);
  timerAttachInterrupt(cpuTickTimer, &cpuTickTimerCallback, true);
  // 1MHz clock, 163 ticks per second desired.
  timerAlarmWrite(cpuTickTimer, 1000000/163, true);
  timerAlarmEnable(cpuTickTimer);
#endif

#if ENABLE_OUTPUTS
  OutputManager::init();
#endif

#if ENABLE_SENSORS
  SensorManager::init();
  S88BusManager::init();
  RemoteSensorManager::init();
#endif

#if LOCONET_ENABLED
  initializeLocoNet();
#endif

  // create OpenMRN executor thread, this can consume near 100% of core 0
  // all tasks must use core 1!
  openmrn->start_executor_thread();

  LOG(INFO, "[OpenMRN] Starting loop task on core:%d", APP_CPU_NUM);
  xTaskCreatePinnedToCore(openmrn_loop_task     // function
                        , "OpenMRN-Loop"        // name
                        , 4096                  // stack
                        , nullptr               // function arg
                        , 1                     // priority
                        , nullptr               // handle
                        , APP_CPU_NUM           // core id
  );

  LOG(INFO, "ESP32 Command Station Started!");
  infoScreen->replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "ESP32-CS Started");

  // put the ESP32 CS main task thread to sleep
  vTaskDelay(portMAX_DELAY);
}