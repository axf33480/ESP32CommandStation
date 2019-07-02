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

using dcc::PacketFlowInterface;
using dcc::RailcomHubFlow;
using dcc::RailcomPrintfFlow;
using dcc::SimpleUpdateLoop;
using openlcb::ConfigDef;
using openlcb::DccAccyConsumer;
using openlcb::Defs;
using openlcb::EventRegistry;
using openlcb::EventRegistryEntry;
using openlcb::EventReport;
using openlcb::Node;
using openlcb::NodeID;
using openlcb::WriteHelper;

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

// ConfigDef comes from LCCCDI.h and is specific to this particular device and
// target. It defines the layout of the configuration memory space and is also
// used to generate the cdi.xml file. Here we instantiate the configuration
// layout. The argument of offset zero is ignored and will be removed later.
static constexpr ConfigDef cfg(0);

std::unique_ptr<Esp32WiFiManager> wifiManager;
std::unique_ptr<RailcomHubFlow> railComHub;
std::unique_ptr<RailcomPrintfFlow> railComDataDumper;
std::unique_ptr<InfoScreen> infoScreen;
std::unique_ptr<InfoScreenStatCollector> infoScreenCollector;
std::unique_ptr<StatusLED> statusLED;
std::unique_ptr<HC12Radio> hc12;
std::unique_ptr<SimpleUpdateLoop> dccUpdateLoop;

std::vector<EventCallbackHandler *> eventCallbacks;

#if LCC_USE_SPIFFS
#define CDI_CONFIG_PREFIX "/spiffs"
#elif LCC_USE_SD
#define CDI_CONFIG_PREFIX "/sdcard"
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

class DccPacketQueueInjector : public PacketFlowInterface {
public:
  void send(Buffer<dcc::Packet> *b, unsigned prio)
  {
    dcc::Packet *pkt = b->data();
    if(pkt->packet_header.send_long_preamble)
    {
      // prog track packet
      dccSignal[DCC_SIGNAL_PROGRAMMING]->loadBytePacket(pkt->payload, pkt->dlc, pkt->packet_header.rept_count);
    }
    else
    {
      // ops track packet
      dccSignal[DCC_SIGNAL_OPERATIONS]->loadBytePacket(pkt->payload, pkt->dlc, pkt->packet_header.rept_count);
      // check if the packet looks like an accessories decoder packet
      if(!pkt->packet_header.is_marklin && pkt->dlc == 2 && pkt->payload[0] & 0x80 && pkt->payload[1] & 0x80) {
        // the second byte of the payload contains part of the address and is stored in ones complement format
        uint8_t onesComplementByteTwo = (pkt->payload[1] ^ 0xF8);
        // decode the accessories decoder address and update the TurnoutManager metadata
        uint16_t boardAddress = (pkt->payload[0] & 0x3F) + ((onesComplementByteTwo >> 4) & 0x07);
        uint8_t boardIndex = ((onesComplementByteTwo >> 1) % 4);
        bool state = onesComplementByteTwo & 0x01;
        // with the board address and index decoded from the packet we can assemble a 12bit decoder address
        uint16_t decoderAddress = (boardAddress * 4 + boardIndex) - 3;
        auto turnout = TurnoutManager::getTurnoutByAddress(decoderAddress);
        if(turnout)
        {
          turnout->set(state, false);
        }
      }
    }
    b->unref();
  }
};

DccPacketQueueInjector dccPacketInjector;

bool otaComplete = false;
esp_ota_handle_t otaInProgress = 0;

#if LCC_CPULOAD_REPORTING
#include <freertos_drivers/arduino/CpuLoad.hxx>

#include <esp_spi_flash.h>
#include <esp32-hal-timer.h>
CpuLoad cpuLogTracker;
hw_timer_t *cpuTickTimer = nullptr;
CpuLoadLog cpuLoadLogger(openmrn.stack()->service());

constexpr uint8_t LCC_CPU_TIMER_NUMBER = 3;
constexpr uint8_t LCC_CPU_TIMER_DIVIDER = 80;

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
#endif // LCC_CPULOAD_REPORTING

void setup() {
  // Setup UART0 115200 8N1 TX: 1, RX: 3, 2k buffer
  uart_config_t uart0 = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_param_config(UART_NUM_0, &uart0);
  uart_driver_install(UART_NUM_0, 2048, 0, 0, NULL, 0);

  LOG(INFO, "\n\nESP32 Command Station v%s starting up", VERSION);

  // set up ADC1 here since we use it for all motor boards
  adc1_config_width(ADC_WIDTH_BIT_12);

  configStore = new ConfigurationManager();

  // pre-create LCC configuration directory
  mkdir(openlcb::CONFIG_DIR, ACCESSPERMS);

#if LCC_FORCE_FACTORY_RESET_ON_STARTUP
  LOG(WARNING, "[LCC] Forcing factory reset");
  unlink(openlcb::CDI_FILENAME);
  unlink(openlcb::CONFIG_FILENAME);
#endif

  openmrn.reset(new OpenMRN(configStore->getNodeId()));

  // init state flow handlers
  infoScreen.reset(new InfoScreen(openmrn->stack()));
  infoScreenCollector.reset(new InfoScreenStatCollector(openmrn->stack()));
  statusLED.reset(new StatusLED(openmrn->stack()));
  hc12.reset(new HC12Radio(openmrn->stack()));

  // Initialize the factory reset helper for the CS
  // this needs to be done after creation of the OpenMRN object
  resetHelper.reset(new FactoryResetHelper());

  // Initialize the WiFi Manager
  configStore->configureWiFi(openmrn->stack(), cfg.seg().wifi());

  // Create the CDI.xml dynamically if it doesn't already exist
  openmrn->create_config_descriptor_xml(cfg, openlcb::CDI_FILENAME);

  // Create the default internal configuration file if it doesn't already exist
  openmrn->stack()->create_config_file_if_needed(cfg.seg().internal_config(),
      openlcb::CANONICAL_VERSION, openlcb::CONFIG_FILE_SIZE);

  // Register Emergency Off event handler (power off)
  eventCallbacks.push_back(new EventCallbackHandler(Defs::EMERGENCY_OFF_EVENT,
                                                    openlcb::CallbackEventHandler::RegistryEntryBits::IS_CONSUMER,
                                                    openmrn->stack()->node(),
                                                    [](const EventRegistryEntry &registry_entry,
                                                       EventReport *report,
                                                       BarrierNotifiable *done)
    {
        // shutdown all track power outputs
        MotorBoardManager::powerOffAll();
    },
    nullptr)
  );

  // Register Clear Emergency Off event handler (power on)
  eventCallbacks.push_back(new EventCallbackHandler(Defs::CLEAR_EMERGENCY_OFF_EVENT,
                                                    openlcb::CallbackEventHandler::RegistryEntryBits::IS_CONSUMER,
                                                    openmrn->stack()->node(),
                                                    [](const EventRegistryEntry &registry_entry,
                                                       EventReport *report,
                                                       BarrierNotifiable *done)
    {
        // Note this will not power on the PROG track as that is only managed via the programming interface
        MotorBoardManager::powerOnAll();
    },
    nullptr)
  );

  // Register Emergency Stop event handler
  eventCallbacks.push_back(new EventCallbackHandler(Defs::EMERGENCY_STOP_EVENT,
                                                    openlcb::CallbackEventHandler::RegistryEntryBits::IS_CONSUMER,
                                                    openmrn->stack()->node(),
                                                    [](const EventRegistryEntry &registry_entry,
                                                       EventReport *report,
                                                       BarrierNotifiable *done)
    {
        LocomotiveManager::emergencyStop();
    },
    nullptr)
  );

  // Create the DCC Event Loop
  dccUpdateLoop.reset(new SimpleUpdateLoop(openmrn->stack()->service(), &dccPacketInjector));

  DCCPPProtocolHandler::init();

  wifiInterface.init();

  gpio_num_t canRXPin, canTXPin;
  if(configStore->needLCCCan(&canRXPin, &canTXPin)) {
    openmrn->add_can_port(new Esp32HardwareCan("esp32can", canRXPin, canTXPin, false));
  }

  nextionInterfaceInit();

  // Start the OpenMRN stack
  openmrn->begin();
  openmrn->start_executor_thread();

#if LCC_CPULOAD_REPORTING
  os_thread_create(&cpuTickTaskHandle, "loadtick", 1, 0, &cpuTickTask, nullptr);
  cpuTickTimer = timerBegin(LCC_CPU_TIMER_NUMBER, LCC_CPU_TIMER_DIVIDER, true);
  timerAttachInterrupt(cpuTickTimer, &cpuTickTimerCallback, true);
  // 1MHz clock, 163 ticks per second desired.
  timerAlarmWrite(cpuTickTimer, 1000000/163, true);
  timerAlarmEnable(cpuTickTimer);
#endif

  MotorBoardManager::registerBoard(MOTORBOARD_CURRENT_SENSE_OPS,
                                   MOTORBOARD_ENABLE_PIN_OPS,
                                   MOTORBOARD_TYPE_OPS,
                                   MOTORBOARD_NAME_OPS);
  MotorBoardManager::registerBoard(MOTORBOARD_CURRENT_SENSE_PROG,
                                   MOTORBOARD_ENABLE_PIN_PROG,
                                   MOTORBOARD_TYPE_PROG,
                                   MOTORBOARD_NAME_PROG,
                                   true);
  dccSignal[DCC_SIGNAL_OPERATIONS] = new SignalGenerator_RMT("OPS", 512, DCC_SIGNAL_OPERATIONS, DCC_SIGNAL_PIN_OPERATIONS, MOTORBOARD_ENABLE_PIN_OPS);
  dccSignal[DCC_SIGNAL_PROGRAMMING] = new SignalGenerator_RMT("PROG", 10, DCC_SIGNAL_PROGRAMMING, DCC_SIGNAL_PIN_PROGRAMMING, MOTORBOARD_ENABLE_PIN_PROG);

  LocomotiveManager::init();
  TurnoutManager::init();

  OutputManager::init();
  SensorManager::init();
  S88BusManager::init();
  RemoteSensorManager::init();

#if ENERGIZE_OPS_TRACK_ON_STARTUP
  MotorBoardManager::powerOnAll();
#else
  MotorBoardManager::powerOffAll();
#endif

#if LOCONET_ENABLED
  initializeLocoNet();
#endif

  LOG(INFO, "[WatchDog] Reconfiguring Timer (15sec)");
  // reset WDT to 15sec
  esp_task_wdt_init(15, true);

  // Enable watchdog timers
  enableLoopWDT();

  LOG(INFO, "ESP32 Command Station Started!");
  infoScreen->replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, "ESP32-CS Started");
}

void loop() {
  openmrn->loop();
  if(otaInProgress && otaComplete) {
    LOG(INFO, "OTA binary has been received, preparing to reboot!");

    // shutdown and cleanup the configuration manager
    delete configStore;

    // wait for the WDT to restart the ESP32
    esp_task_wdt_init(1, true);
    while(true);
  }
  MotorBoardManager::check();
}

void dumpTaskList()
{
#if configUSE_TRACE_FACILITY
  UBaseType_t taskCount = uxTaskGetNumberOfTasks();
  std::unique_ptr<TaskStatus_t[]> taskList(new TaskStatus_t[taskCount]);
  uint32_t ulTotalRunTime;
  UBaseType_t retrievedTaskCount = uxTaskGetSystemState(taskList.get(), taskCount, &ulTotalRunTime);
  printf("time[us]: %" PRIu64 " core: %d, freeHeap: %u, largest: %u\n",
          esp_timer_get_time(), xPortGetCoreID(), heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
          heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

#if configTASKLIST_INCLUDE_COREID
  printf("%15s%10s%10s%10s%10s%10s%10s%10s%10s\n",
          "NAME", "ID", "STATE", "PRIO", "BASE", "TIME", "CPU", "STACK", "CORE");
#else
  printf("%15s%10s%10s%10s%10s%10s%10s%10s\n",
          "NAME", "ID", "STATE", "PRIO", "BASE", "TIME", "CPU", "STACK");
#endif // configTASKLIST_INCLUDE_COREID

  for (int task = 0; task < retrievedTaskCount; task++)
  {
#if configTASKLIST_INCLUDE_COREID
    printf("%15s%10d%10s%10d%10d%10" PRIu64 "%10.2f%10d%10d\n"
#else
    printf("%15s%10d%10s%10d%10d%10" PRIu64 "%10.2f%10d\n"
#endif // configTASKLIST_INCLUDE_COREID
           , taskList[task].pcTaskName
           , taskList[task].xTaskNumber
           , taskList[task].eCurrentState == eRunning ? "Running" : 
             taskList[task].eCurrentState == eReady ? "Ready" : 
             taskList[task].eCurrentState == eBlocked ? "Blocked" : 
             taskList[task].eCurrentState == eSuspended ? "Suspended" : 
             taskList[task].eCurrentState == eDeleted ? "Deleted" : "Unknown"
           , taskList[task].uxCurrentPriority
           , taskList[task].uxBasePriority
           , taskList[task].ulRunTimeCounter
           , (float)taskList[task].ulRunTimeCounter / (float)ulTotalRunTime
           , taskList[task].usStackHighWaterMark
#if configTASKLIST_INCLUDE_COREID
           , taskList[task].xCoreID
#endif // configTASKLIST_INCLUDE_COREID
          );
  }
#endif // configUSE_TRACE_FACILITY
}