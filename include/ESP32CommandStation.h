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

#pragma once

/////////////////////////////////////////////////////////////////////////////////////
// RELEASE VERSION
/////////////////////////////////////////////////////////////////////////////////////

#define VERSION "1.4.0"

#include <algorithm>
#include <functional>
#include <string>
#include <sstream>
#include <vector>

#include <driver/uart.h>

#include <Arduino.h>
#include <ArduinoJson.h>
#include <StringArray.h>

#include <OpenMRNLite.h>

#include <dcc/Loco.hxx>
#include <dcc/Packet.hxx>
#include <dcc/PacketFlowInterface.hxx>
#include <dcc/RailcomHub.hxx>
#include <dcc/RailcomPortDebug.hxx>
#include <dcc/SimpleUpdateLoop.hxx>

#include <openlcb/CallbackEventHandler.hxx>
#include <openlcb/ConfiguredTcpConnection.hxx>
#include <openlcb/DccAccyConsumer.hxx>
#include <openlcb/DccAccyProducer.hxx>
#include <openlcb/TcpDefs.hxx>

#include <os/MDNS.hxx>

#include <utils/format_utils.hxx>
#include <utils/logging.h>
#include <utils/macros.h>
#include <utils/StringPrintf.hxx>

#include <ESPAsyncWebServer.h>

#include <esp_ota_ops.h>

#include "Config.h"

using dcc::PacketFlowInterface;
using dcc::RailcomHubFlow;
using dcc::RailcomPrintfFlow;
using dcc::SimpleUpdateLoop;
using openlcb::BitEventProducer;
using openlcb::CallbackEventHandler;
using openlcb::DccAccyConsumer;
using openlcb::Defs;
using openlcb::EventId;
using openlcb::EventRegistry;
using openlcb::EventRegistryEntry;
using openlcb::EventReport;
using openlcb::MemoryBit;
using openlcb::Node;
using openlcb::NodeID;
using openlcb::SimpleCanStack;
using openlcb::WriteHelper;

using std::unique_ptr;
using std::vector;
using std::string;

// Simplified callback handler to automatically register the callbacks
class EventCallbackHandler : public CallbackEventHandler {
public:
  EventCallbackHandler(EventId eventID,
                       uint32_t callbackType,
                       Node *node,
                       CallbackEventHandler::EventReportHandlerFn report_handler,
                       CallbackEventHandler::EventStateHandlerFn state_handler) :
    CallbackEventHandler(node, report_handler, state_handler)
  {
    add_entry(eventID, callbackType);
  }
};

/////////////////////////////////////////////////////////////////////////////////////
//
// The following parameters define how many preamble bits will be transmitted as part
// of the DCC packet to the track. For some older sound decodes it may be necessary
// to increase from 22 bits on the PROG track to 30 or even 40.
//
// The maximum number of preamble bits is 50. For OPS the minimum to send is 11 but
// 16 is recommended for RailCom support.

#define OPS_TRACK_PREAMBLE_BITS 16
#define PROG_TRACK_PREAMBLE_BITS 22

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE WHICH PINS ARE USED FOR OPS RAILCOM DETECTION
//
#define OPS_BRAKE_ENABLE_PIN NOT_A_PIN
#define OPS_RAILCOM_ENABLE_PIN NOT_A_PIN
#define OPS_RAILCOM_SHORT_PIN NOT_A_PIN
#define OPS_RAILCOM_UART 2
#define OPS_RAILCOM_UART_RX_PIN NOT_A_PIN

#ifndef STATUS_LED_ENABLED
#define STATUS_LED_ENABLED false
#endif

#ifndef OPS_TRACK_PREAMBLE_BITS
#define OPS_TRACK_PREAMBLE_BITS 16
#endif

#ifndef PROG_TRACK_PREAMBLE_BITS
#define PROG_TRACK_PREAMBLE_BITS 22
#endif

#if OPS_TRACK_PREAMBLE_BITS < 11
#error "OPS_TRACK_PREAMBLE_BITS is too low, a minimum of 11 bits must be transmitted for the DCC decoder to accept the packets."
#endif

#if OPS_TRACK_PREAMBLE_BITS > 20
#error "OPS_TRACK_PREAMBLE_BITS is too high. The OPS track only supports up to 20 preamble bits."
#endif

#if PROG_TRACK_PREAMBLE_BITS < 22
#error "PROG_TRACK_PREAMBLE_BITS is too low, a minimum of 22 bits must be transmitted for reliability on the PROG track."
#endif

#if OPS_TRACK_PREAMBLE_BITS > 50
#error "PROG_TRACK_PREAMBLE_BITS is too high. The PROG track only supports up to 50 preamble bits."
#endif

// initialize default values for various pre-compiler checks to simplify logic in a lot of places
#if (defined(INFO_SCREEN_LCD) && INFO_SCREEN_LCD) || (defined(INFO_SCREEN_OLED) && INFO_SCREEN_OLED)
#define INFO_SCREEN_ENABLED true
#if (defined(INFO_SCREEN_LCD) && INFO_SCREEN_LCD)
#define INFO_SCREEN_OLED false
#endif
#if (defined(INFO_SCREEN_OLED) && INFO_SCREEN_OLED)
#define INFO_SCREEN_LCD false
#endif
#else
#define INFO_SCREEN_ENABLED false
#define INFO_SCREEN_LCD false
#define INFO_SCREEN_OLED false
#endif

#ifndef NEXTION_ENABLED
#define NEXTION_ENABLED false
#endif

#ifndef HC12_RADIO_ENABLED
#define HC12_RADIO_ENABLED false
#endif

#ifndef LCC_FORCE_FACTORY_RESET_ON_STARTUP 
#define LCC_FORCE_FACTORY_RESET_ON_STARTUP false
#endif

#ifndef LOCONET_ENABLED
#define LOCONET_ENABLED false
#endif

#ifndef S88_ENABLED
#define S88_ENABLED false
#endif

#ifndef ENERGIZE_OPS_TRACK_ON_STARTUP
#define ENERGIZE_OPS_TRACK_ON_STARTUP false
#endif

#ifndef LOCONET_INVERTED_LOGIC
#define LOCONET_INVERTED_LOGIC false
#endif

#ifndef LOCONET_ENABLE_RX_PIN_PULLUP
#define LOCONET_ENABLE_RX_PIN_PULLUP false
#endif

#ifndef WIFI_ENABLE_SOFT_AP
#define WIFI_ENABLE_SOFT_AP false
#endif

#ifndef WIFI_SOFT_AP_CHANNEL
#define WIFI_SOFT_AP_CHANNEL 6
#endif

#ifndef WIFI_SOFT_AP_MAX_CLIENTS
#define WIFI_SOFT_AP_MAX_CLIENTS 4
#endif

#ifndef STATUS_LED_ENABLED
#define STATUS_LED_TYPE WS281X_800
#define STATUS_LED_COLOR_ORDER RGB
#endif

/////////////////////////////////////////////////////////////////////////////////////
// S88 Maximum sensors per bus.
/////////////////////////////////////////////////////////////////////////////////////
constexpr uint16_t S88_MAX_SENSORS_PER_BUS = 512;
#ifndef S88_FIRST_SENSOR
#define S88_FIRST_SENSOR S88_MAX_SENSORS_PER_BUS
#endif

#include "JsonConstants.h"
#include "ConfigurationManager.h"

#include "dcc/DCCSignalGenerator.h"
#include "dcc/DCCSignalGenerator_RMT.h"
#include "dcc/DCCProgrammer.h"
#include "dcc/Locomotive.h"
#include "dcc/Turnouts.h"

#include "interfaces/DCCppProtocol.h"
#include "interfaces/NextionInterface.h"
#include "interfaces/WiFiInterface.h"

#include "stateflows/InfoScreen.h"
#include "stateflows/InfoScreenCollector.h"
#include "stateflows/MonitoredHBridge.h"
#include "stateflows/StatusLED.h"
#include "stateflows/HC12Radio.h"
#include "stateflows/OTAMonitor.h"

#include "io/Outputs.h"
#include "io/Sensors.h"
#include "io/S88Sensors.h"
#include "io/RemoteSensors.h"

extern vector<uint8_t> restrictedPins;
extern unique_ptr<Esp32WiFiManager> wifiManager;
extern unique_ptr<dcc::RailcomHubFlow> railComHub;
extern unique_ptr<dcc::RailcomPrintfFlow> railComDataDumper;
extern unique_ptr<InfoScreenStatCollector> infoScreenCollector;
extern unique_ptr<StatusLED> statusLED;
extern unique_ptr<HC12Radio> hc12;
extern unique_ptr<OTAMonitorFlow> otaMonitor;

void register_monitored_hbridge(SimpleCanStack *, const adc1_channel_t, const gpio_num_t, const gpio_num_t,
                                const uint32_t, const uint32_t, const string &, const string &,
                                const TrackOutputConfig &, const bool=false);

void get_hbridge_status_json(JsonArray);
bool is_track_power_on();
void enable_all_hbridges();
void enable_named_hbridge(string);
void disable_all_hbridges();
void disable_named_hbridge(string);
uint32_t get_hbridge_sample(string);
void broadcast_all_hbridge_statuses();
void broadcast_named_hbridge_status(string);
string get_hbridge_info_screen_data();

#if LOCONET_ENABLED
#include <LocoNetESP32UART.h>
extern LocoNetESP32Uart locoNet;
#endif

void initializeLocoNet();

#define MUTEX_LOCK(mutex)    do {} while (xSemaphoreTake(mutex, portMAX_DELAY) != pdPASS)
#define MUTEX_UNLOCK(mutex)  xSemaphoreGive(mutex)

/////////////////////////////////////////////////////////////////////////////////////
// Ensure SSID and PASSWORD are provided.
/////////////////////////////////////////////////////////////////////////////////////
#if !defined(SSID_NAME) || !defined(SSID_PASSWORD)
#error "Invalid Configuration detected, Config_WiFi.h is a mandatory module."
#endif

/////////////////////////////////////////////////////////////////////////////////////
// Ensure the required h-bridge parameters are specified and not overlapping.
/////////////////////////////////////////////////////////////////////////////////////
#if !defined(OPS_HBRIDGE_NAME) || \
    !defined(OPS_HBRIDGE_ENABLE_PIN) || \
    !defined(OPS_HBRIDGE_THERMAL_PIN) || \
    !defined(OPS_HBRIDGE_CURRENT_SENSE_ADC) || \
    !defined(OPS_HBRIDGE_TYPE) || \
    !defined(PROG_HBRIDGE_NAME) || \
    !defined(PROG_HBRIDGE_ENABLE_PIN) || \
    !defined(PROG_HBRIDGE_CURRENT_SENSE_ADC) || \
    !defined(PROG_HBRIDGE_TYPE) || \
    !defined(DCC_SIGNAL_PIN_OPERATIONS) || \
    !defined(DCC_SIGNAL_PIN_PROGRAMMING)
#error "Invalid Configuration detected, Config_MotorBoard.h is a mandatory module."
#endif

#if OPS_HBRIDGE_TYPE == L298
#define OPS_HBRIDGE_MAX_MILIAMPS 2000
#define OPS_HBRIDGE_LIMIT_MILIAMPS 2000
#define OPS_HBRIDGE_TYPE_NAME "L298"
#elif OPS_HBRIDGE_TYPE == LMD18200
#define OPS_HBRIDGE_MAX_MILIAMPS 3000
#define OPS_HBRIDGE_LIMIT_MILIAMPS 3000
#define OPS_HBRIDGE_TYPE_NAME "LMD18200"
#elif OPS_HBRIDGE_TYPE == POLOLU
#define OPS_HBRIDGE_MAX_MILIAMPS 2500
#define OPS_HBRIDGE_LIMIT_MILIAMPS 2500
#define OPS_HBRIDGE_TYPE_NAME "POLOLU"
#elif OPS_HBRIDGE_TYPE == BTS7960B_5A
#define OPS_HBRIDGE_MAX_MILIAMPS 43000
#define OPS_HBRIDGE_LIMIT_MILIAMPS 5000
#define OPS_HBRIDGE_TYPE_NAME "BTS7960B"
#elif OPS_HBRIDGE_TYPE == BTS7960B_10A
#define OPS_HBRIDGE_MAX_MILIAMPS 43000
#define OPS_HBRIDGE_LIMIT_MILIAMPS 10000
#define OPS_HBRIDGE_TYPE_NAME "BTS7960B"
#endif

#if PROG_HBRIDGE_TYPE == L298
#define PROG_HBRIDGE_MAX_MILIAMPS 2000
#define PROG_HBRIDGE_TYPE_NAME "L298"
#elif PROG_HBRIDGE_TYPE == LMD18200
#define PROG_HBRIDGE_MAX_MILIAMPS 3000
#define PROG_HBRIDGE_TYPE_NAME "LMD18200"
#elif PROG_HBRIDGE_TYPE == POLOLU
#define PROG_HBRIDGE_MAX_MILIAMPS 2500
#define PROG_HBRIDGE_TYPE_NAME "POLOLU"
#elif PROG_HBRIDGE_TYPE == BTS7960B_5A
#define PROG_HBRIDGE_MAX_MILIAMPS 43000
#define PROG_HBRIDGE_TYPE_NAME "BTS7960B"
#elif PROG_HBRIDGE_TYPE == BTS7960B_10A
#define PROG_HBRIDGE_MAX_MILIAMPS 43000
#define PROG_HBRIDGE_TYPE_NAME "BTS7960B"
#endif
// programming track is current limited internally by the hbridge monitor code
#define PROG_HBRIDGE_LIMIT_MILIAMPS PROG_HBRIDGE_MAX_MILIAMPS

#if OPS_HBRIDGE_ENABLE_PIN == PROG_HBRIDGE_ENABLE_PIN
#error "Invalid Configuration detected, OPS_HBRIDGE_ENABLE_PIN and PROG_HBRIDGE_ENABLE_PIN must be unique."
#endif

#if DCC_SIGNAL_PIN_OPERATIONS == DCC_SIGNAL_PIN_PROGRAMMING
#error "Invalid Configuration detected, DCC_SIGNAL_PIN_OPERATIONS and DCC_SIGNAL_PIN_PROGRAMMING must be unique."
#endif

#if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == OPS_HBRIDGE_ENABLE_PIN
#error "Invalid Configuration detected, STATUS_LED_DATA_PIN and OPS_HBRIDGE_ENABLE_PIN must be unique."
#endif

#if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == PROG_HBRIDGE_ENABLE_PIN
#error "Invalid Configuration detected, STATUS_LED_DATA_PIN and PROG_HBRIDGE_ENABLE_PIN must be unique."
#endif

#if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == DCC_SIGNAL_PIN_OPERATIONS
#error "Invalid Configuration detected, STATUS_LED_DATA_PIN and DCC_SIGNAL_PIN_OPERATIONS must be unique."
#endif

#if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == DCC_SIGNAL_PIN_PROGRAMMING
#error "Invalid Configuration detected, STATUS_LED_DATA_PIN and DCC_SIGNAL_PIN_PROGRAMMING must be unique."
#endif

/////////////////////////////////////////////////////////////////////////////////////
// Ensure either OLED or LCD display is active and not both.
/////////////////////////////////////////////////////////////////////////////////////
#if defined(INFO_SCREEN_OLED) && INFO_SCREEN_OLED && defined(INFO_SCREEN_LCD) && INFO_SCREEN_LCD
#error "Invalid Configuration detected, it is not supported to include both OLED and LCD support."
#endif

/////////////////////////////////////////////////////////////////////////////////////
// Nextion interface configuration validations
/////////////////////////////////////////////////////////////////////////////////////
#if NEXTION_ENABLED
  #if NEXTION_UART_RX_PIN == NEXTION_UART_TX_PIN
  #error "Invalid Configuration detected, NEXTION_UART_RX_PIN and NEXTION_UART_TX_PIN must be unique."
  #endif
  #if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == NEXTION_UART_RX_PIN
  #error "Invalid Configuration detected, STATUS_LED_DATA_PIN and NEXTION_UART_RX_PIN must be unique."
  #endif
  #if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == NEXTION_UART_TX_PIN
  #error "Invalid Configuration detected, STATUS_LED_DATA_PIN and NEXTION_UART_TX_PIN must be unique."
  #endif
  #if HC12_RADIO_ENABLED
    #if NEXTION_UART_NUM == HC12_UART_NUM
    #error "Invalid Configuration detected, the Nextion and HC12 can not share the UART interface."
    #endif
    #if NEXTION_UART_RX_PIN == HC12_RX_PIN
    #error "Invalid Configuration detected, the Nextion and HC12 can not share the same RX Pin."
    #endif
    #if NEXTION_UART_TX_PIN == HC12_TX_PIN
    #error "Invalid Configuration detected, the Nextion and HC12 can not share the same TX Pin."
    #endif
  #endif
  #if LOCONET_ENABLED
    #if NEXTION_UART_NUM == LOCONET_UART
    #error "Invalid Configuration detected, the Nextion and LocoNet can not share the UART interface."
    #endif
    #if NEXTION_UART_RX_PIN == LOCONET_RX_PIN
    #error "Invalid Configuration detected, the Nextion and LocoNet can not share the same RX Pin."
    #endif
    #if NEXTION_UART_TX_PIN == LOCONET_TX_PIN
    #error "Invalid Configuration detected, the Nextion and LocoNet can not share the same TX Pin."
    #endif
  #endif
  #if S88_ENABLED
    #if S88_CLOCK_PIN == NEXTION_UART_RX_PIN
    #error "Invalid Configuration detected, the Nextion RX Pin and S88_CLOCK_PIN must be unique."
    #endif
    #if S88_CLOCK_PIN == NEXTION_UART_TX_PIN
    #error "Invalid Configuration detected, the NEXTION_TX_PIN and S88_CLOCK_PIN must be unique."
    #endif
    #if S88_RESET_PIN == NEXTION_UART_RX_PIN
    #error "Invalid Configuration detected, the Nextion RX Pin and S88_RESET_PIN must be unique."
    #endif
    #if S88_RESET_PIN == NEXTION_UART_TX_PIN
    #error "Invalid Configuration detected, the NEXTION_TX_PIN and S88_RESET_PIN must be unique."
    #endif
    #if S88_LOAD_PIN == NEXTION_UART_RX_PIN
    #error "Invalid Configuration detected, the Nextion RX Pin and S88_LOAD_PIN must be unique."
    #endif
    #if S88_LOAD_PIN == NEXTION_UART_TX_PIN
    #error "Invalid Configuration detected, the NEXTION_TX_PIN and S88_LOAD_PIN must be unique."
    #endif
  #endif
#endif

/////////////////////////////////////////////////////////////////////////////////////
// HC12 Radio interface configuration validations
/////////////////////////////////////////////////////////////////////////////////////
#if HC12_RADIO_ENABLED
  #if HC12_RX_PIN == HC12_TX_PIN
  #error "Invalid Configuration detected, HC12_RX_PIN and HC12_TX_PIN must be unique."
  #endif
  #if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == HC12_RX_PIN
  #error "Invalid Configuration detected, STATUS_LED_DATA_PIN and NEXTION_TX_PIN must be unique."
  #endif
  #if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == HC12_TX_PIN
  #error "Invalid Configuration detected, STATUS_LED_DATA_PIN and HC12_TX_PIN must be unique."
  #endif
  #if S88_ENABLED
    #if S88_CLOCK_PIN == HC12_RX_PIN
    #error "Invalid Configuration detected, the HC12_RX_PIN and S88_CLOCK_PIN must be unique."
    #endif
    #if S88_CLOCK_PIN == HC12_TX_PIN
    #error "Invalid Configuration detected, the HC12_TX_PIN and S88_CLOCK_PIN must be unique."
    #endif
    #if S88_RESET_PIN == HC12_RX_PIN
    #error "Invalid Configuration detected, the HC12_RX_PIN and S88_RESET_PIN must be unique."
    #endif
    #if S88_RESET_PIN == HC12_TX_PIN
    #error "Invalid Configuration detected, the HC12_TX_PIN and S88_RESET_PIN must be unique."
    #endif
    #if S88_LOAD_PIN == HC12_RX_PIN
    #error "Invalid Configuration detected, the HC12_RX_PIN and S88_LOAD_PIN must be unique."
    #endif
    #if S88_LOAD_PIN == HC12_TX_PIN
    #error "Invalid Configuration detected, the HC12_TX_PIN and S88_LOAD_PIN must be unique."
    #endif
  #endif
  #if LOCONET_ENABLED
    #if LOCONET_UART == HC12_UART_NUM
    #error "Invalid Configuration detected, the LocoNet and HC12 can not share the UART interface."
    #endif
    #if LOCONET_RX_PIN == HC12_RX_PIN
    #error "Invalid Configuration detected, the LOCONET_RX_PIN and HC12_RX_PIN must be unique."
    #endif
    #if LOCONET_TX_PIN == HC12_TX_PIN
    #error "Invalid Configuration detected, the LOCONET_TX_PIN and HC12_TX_PIN must be unique."
    #endif
  #endif
#endif

/////////////////////////////////////////////////////////////////////////////////////
// LocoNet interface configuration validations
/////////////////////////////////////////////////////////////////////////////////////
#if LOCONET_ENABLED
  #if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == LOCONET_RX_PIN
  #error "Invalid Configuration detected, STATUS_LED_DATA_PIN and LOCONET_RX_PIN must be unique."
  #endif
  #if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == LOCONET_TX_PIN
  #error "Invalid Configuration detected, STATUS_LED_DATA_PIN and LOCONET_TX_PIN must be unique."
  #endif
  #if S88_ENABLED
    #if S88_CLOCK_PIN == LOCONET_RX_PIN
    #error "Invalid Configuration detected, the LOCONET_RX_PIN and S88_CLOCK_PIN must be unique."
    #endif
    #if S88_CLOCK_PIN == LOCONET_TX_PIN
    #error "Invalid Configuration detected, the LOCONET_TX_PIN and S88_CLOCK_PIN must be unique."
    #endif
    #if S88_RESET_PIN == LOCONET_RX_PIN
    #error "Invalid Configuration detected, the LOCONET_RX_PIN and S88_RESET_PIN must be unique."
    #endif
    #if S88_RESET_PIN == LOCONET_TX_PIN
    #error "Invalid Configuration detected, the LOCONET_TX_PIN and S88_RESET_PIN must be unique."
    #endif
    #if S88_LOAD_PIN == LOCONET_RX_PIN
    #error "Invalid Configuration detected, the LOCONET_RX_PIN and S88_LOAD_PIN must be unique."
    #endif
    #if S88_LOAD_PIN == LOCONET_TX_PIN
    #error "Invalid Configuration detected, the LOCONET_TX_PIN and S88_LOAD_PIN must be unique."
    #endif
  #endif
#endif

/////////////////////////////////////////////////////////////////////////////////////
// LCC interface configuration validations
/////////////////////////////////////////////////////////////////////////////////////
#if LCC_CAN_RX_PIN != NOT_A_PIN
  #if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == LCC_CAN_RX_PIN
  #error "Invalid Configuration detected, STATUS_LED_DATA_PIN and LCC_CAN_RX_PIN must be unique."
  #endif
  #if S88_ENABLED
    #if S88_CLOCK_PIN == LCC_CAN_RX_PIN
    #error "Invalid Configuration detected, LCC_CAN_RX_PIN and S88_CLOCK_PIN must be unique."
    #endif
    #if S88_CLOCK_PIN == LCC_CAN_RX_PIN
    #error "Invalid Configuration detected, LCC_CAN_RX_PIN and S88_CLOCK_PIN must be unique."
    #endif
    #if S88_RESET_PIN == LCC_CAN_RX_PIN
    #error "Invalid Configuration detected, LCC_CAN_RX_PIN and S88_CLOCK_PIN must be unique."
    #endif
  #endif
  #if LOCONET_ENABLED
    #if LCC_CAN_RX_PIN == LOCONET_RX_PIN
    #error "Invalid Configuration detected, LCC_CAN_RX_PIN and LOCONET_RX_PIN must be unique."
    #endif
    #if LCC_CAN_RX_PIN == LOCONET_TX_PIN
    #error "Invalid Configuration detected, LCC_CAN_RX_PIN and LOCONET_TX_PIN must be unique."
    #endif
  #endif
  #if NEXTION_ENABLED
    #if LCC_CAN_RX_PIN == NEXTION_RX_PIN
    #error "Invalid Configuration detected, LCC_CAN_RX_PIN and NEXTION_RX_PIN must be unique."
    #endif
    #if LCC_CAN_RX_PIN == NEXTION_TX_PIN
    #error "Invalid Configuration detected, LCC_CAN_RX_PIN and NEXTION_TX_PIN must be unique."
    #endif
  #endif
#endif
#if LCC_CAN_TX_PIN != NOT_A_PIN
  #if STATUS_LED_ENABLED && STATUS_LED_DATA_PIN == LCC_CAN_TX_PIN
  #error "Invalid Configuration detected, STATUS_LED_DATA_PIN and LCC_CAN_TX_PIN must be unique."
  #endif
  #if S88_ENABLED
    #if S88_CLOCK_PIN == LCC_CAN_TX_PIN
    #error "Invalid Configuration detected, LCC_CAN_TX_PIN and S88_CLOCK_PIN must be unique."
    #endif
    #if S88_CLOCK_PIN == LCC_CAN_TX_PIN
    #error "Invalid Configuration detected, LCC_CAN_TX_PIN and S88_CLOCK_PIN must be unique."
    #endif
    #if S88_RESET_PIN == LCC_CAN_TX_PIN
    #error "Invalid Configuration detected, LCC_CAN_TX_PIN and S88_CLOCK_PIN must be unique."
    #endif
  #endif
  #if LOCONET_ENABLED
    #if LCC_CAN_TX_PIN == LOCONET_RX_PIN
    #error "Invalid Configuration detected, LCC_CAN_TX_PIN and LOCONET_RX_PIN must be unique."
    #endif
    #if LCC_CAN_TX_PIN == LOCONET_TX_PIN
    #error "Invalid Configuration detected, LCC_CAN_TX_PIN and LOCONET_TX_PIN must be unique."
    #endif
  #endif
  #if NEXTION_ENABLED
    #if LCC_CAN_TX_PIN == NEXTION_RX_PIN
    #error "Invalid Configuration detected, LCC_CAN_TX_PIN and NEXTION_RX_PIN must be unique."
    #endif
    #if LCC_CAN_TX_PIN == NEXTION_TX_PIN
    #error "Invalid Configuration detected, LCC_CAN_TX_PIN and NEXTION_TX_PIN must be unique."
    #endif
  #endif
#endif
#if LCC_CAN_RX_PIN == LCC_CAN_TX_PIN && LCC_CAN_RX_PIN != NOT_A_PIN && LCC_CAN_TX_PIN != NOT_A_PIN
  #error "Invalid Configuration detected, LCC_CAN_RX_PIN and LCC_CAN_TX_PIN must be unique."
#endif
