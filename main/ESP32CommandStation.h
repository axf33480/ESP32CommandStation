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

#include <dcc/LocalTrackIf.hxx>
#include <dcc/Loco.hxx>
#include <dcc/Packet.hxx>
#include <dcc/SimpleUpdateLoop.hxx>

#include <esp_image_format.h>

#include <openlcb/SimpleStack.hxx>

#include <os/MDNS.hxx>
#include <os/OS.hxx>

#include <StatusLED.h>

#include <utils/AutoSyncFileFlow.hxx>
#include <utils/constants.hxx>
#include <utils/FileUtils.hxx>
#include <utils/format_utils.hxx>
#include <utils/logging.h>
#include <utils/macros.h>
#include <utils/StringPrintf.hxx>
#include <utils/Uninitialized.hxx>

#include "sdkconfig.h"

// Include ESP32 Command Station component declarations
#include "JsonConstants.h"
#include "ConfigurationManager.h"
#include "ESP32TrainDatabase.h"
#include "HttpStringUtils.h"

#include <DCCProgrammer.h>
#include <Turnouts.h>

#include <DCCppProtocol.h>

#if CONFIG_NEXTION
#include "Interfaces/nextion/NextionInterface.h"
#endif

extern uninitialized<dcc::LocalTrackIf> trackInterface;
extern std::unique_ptr<openlcb::SimpleCanStack> lccStack;

#if LOCONET_ENABLED
#include <LocoNetESP32UART.h>
extern LocoNetESP32Uart locoNet;
#endif

void initializeLocoNet();

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