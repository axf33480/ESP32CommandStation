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

#include <dcc/Loco.hxx>
#include <dcc/Packet.hxx>
#include <dcc/SimpleUpdateLoop.hxx>

#include <esp_image_format.h>

#include <openlcb/SimpleStack.hxx>

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

#include "sdkconfig.h"

// Include ESP32 Command Station component declarations
#include "JsonConstants.h"
#include <ConfigurationManager.h>
#include "ESP32TrainDatabase.h"
#include <HttpStringUtils.h>
#include <DCCProgrammer.h>
#include <DCCppProtocol.h>
#include <StatusLED.h>
#include <Turnouts.h>

#if CONFIG_NEXTION
#include "Interfaces/nextion/NextionInterface.h"
#endif

#if LOCONET_ENABLED
#include <LocoNetESP32UART.h>
extern LocoNetESP32Uart locoNet;
void initializeLocoNet();
#endif

#endif // ESP32_CS_H_