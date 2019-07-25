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
std::vector<uint8_t> restrictedPins
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
std::vector<uint8_t> restrictedPins;
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

// state flows that are not defined elsewhere
unique_ptr<FreeRTOSTaskMonitor> taskMonitor;
unique_ptr<LCCStatCollector> lccStatCollector;
unique_ptr<OTAMonitorFlow> otaMonitor;

unique_ptr<SimpleUpdateLoop> dccUpdateLoop;
unique_ptr<RailcomHubFlow> railComHub;
unique_ptr<RailcomPrintfFlow> railComDataDumper;

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

  LOG(INFO
    , "\n\nESP32 Command Station v%s starting up"
    , ESP32CS_VERSION);

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

  // Create the CDI.xml dynamically if it doesn't already exist.
  openmrn->create_config_descriptor_xml(cfg, openlcb::CDI_FILENAME);

  // Create the default internal configuration file if it doesn't already exist.
  openmrn->stack()->create_config_file_if_needed(cfg.seg().internal_config()
                                               , ESP32CS_NUMERIC_VERSION
                                               , openlcb::CONFIG_FILE_SIZE);

  // Initialize the DCC Signal Generators.
  dccSignal[DCC_SIGNAL_OPERATIONS].reset(
    new SignalGenerator_RMT(OPS_HBRIDGE_NAME           // name
                          , 512                        // packet queue size
                          , DCC_SIGNAL_OPERATIONS      // signal ID
                          , DCC_SIGNAL_PIN_OPERATIONS  // signal pin
                          , OPS_HBRIDGE_ENABLE_PIN));  // h-bridge enable pin
  dccSignal[DCC_SIGNAL_PROGRAMMING].reset(
    new SignalGenerator_RMT(PROG_HBRIDGE_NAME           // name
                          , 10                          // packet queue size
                          , DCC_SIGNAL_PROGRAMMING      // signal ID
                          , DCC_SIGNAL_PIN_PROGRAMMING  // signal pin
                          , PROG_HBRIDGE_ENABLE_PIN));  // h-bridge enable pin

  // Create the DCC Update Loop.
  dccUpdateLoop.reset(
    new SimpleUpdateLoop(openmrn->stack()->service()
                       , dccSignal[DCC_SIGNAL_OPERATIONS].get()));

  DCCPPProtocolHandler::init();

  wifiInterface.init();

  setup_hbridge_event_handlers(openmrn->stack()->node());

  register_monitored_hbridge(openmrn->stack()
                          , (adc1_channel_t)OPS_HBRIDGE_CURRENT_SENSE_ADC
                          , (gpio_num_t)OPS_HBRIDGE_ENABLE_PIN
                          , (gpio_num_t)OPS_HBRIDGE_THERMAL_PIN
                          , OPS_HBRIDGE_LIMIT_MILIAMPS
                          , OPS_HBRIDGE_MAX_MILIAMPS
                          , OPS_HBRIDGE_NAME
                          , OPS_HBRIDGE_TYPE_NAME
                          , cfg.seg().hbridge().entry(0));

  register_monitored_hbridge(openmrn->stack()
                          , (adc1_channel_t)PROG_HBRIDGE_CURRENT_SENSE_ADC
                          , (gpio_num_t)PROG_HBRIDGE_ENABLE_PIN
                          , (gpio_num_t)NOT_A_PIN
                          , PROG_HBRIDGE_LIMIT_MILIAMPS
                          , PROG_HBRIDGE_MAX_MILIAMPS
                          , PROG_HBRIDGE_NAME
                          , PROG_HBRIDGE_TYPE_NAME
                          , cfg.seg().hbridge().entry(1)
                          , true);

  // Initialize the CAN interface (if configured).
  configStore->configureCAN(openmrn.get());

  nextionInterfaceInit();

  LocomotiveManager::init(openmrn->stack()->node());

  // Initialize the turnout manager and register it with the LCC stack to
  // process accessories packets.
  turnoutManager.reset(new TurnoutManager(openmrn->stack()->node()));

  // Start the OpenMRN stack.
  openmrn->begin();

  // create OpenMRN executor thread, this will consume near 100% of core 0
  // all tasks must use core 1!
  openmrn->start_executor_thread();

  LOG(INFO, "[OpenMRN] Starting loop task on core:%d", APP_CPU_NUM);
  xTaskCreatePinnedToCore(openmrn_loop_task     // function
                        , "OpenMRN-Loop"        // name
                        , 2048                  // stack
                        , nullptr               // function arg
                        , 1                     // priority
                        , nullptr               // handle
                        , APP_CPU_NUM           // core id
  );

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

  LOG(INFO, "ESP32 Command Station Started!");
  infoScreen->replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "ESP32-CS Started");
}