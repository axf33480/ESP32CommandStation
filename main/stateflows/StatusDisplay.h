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

#ifndef STATUS_DISPLAY_H_
#define STATUS_DISPLAY_H_

#include <string>
#include <executor/StateFlow.hxx>
#include <openlcb/SimpleStack.hxx>
#include "stateflows/LCCStatCollector.h"

#if CONFIG_DISPLAY_TYPE_OLED
#define INFO_SCREEN_STATION_INFO_LINE 0
#define INFO_SCREEN_IP_ADDR_LINE 1
#define INFO_SCREEN_CLIENTS_LINE 2
#define INFO_SCREEN_TRACK_POWER_LINE 3
#define INFO_SCREEN_ROTATING_STATUS_LINE 4
#elif CONFIG_DISPLAY_TYPE_LCD && CONFIG_DISPLAY_LCD_LINE_COUNT > 2
#define INFO_SCREEN_STATION_INFO_LINE 0
#define INFO_SCREEN_IP_ADDR_LINE 1
#define INFO_SCREEN_CLIENTS_LINE -1
#define INFO_SCREEN_TRACK_POWER_LINE 2
#define INFO_SCREEN_ROTATING_STATUS_LINE 3
#else
#define INFO_SCREEN_STATION_INFO_LINE 0
#define INFO_SCREEN_IP_ADDR_LINE 0
#define INFO_SCREEN_CLIENTS_LINE -1
#define INFO_SCREEN_TRACK_POWER_LINE -1
#define INFO_SCREEN_ROTATING_STATUS_LINE 1
#endif

class StatusDisplay : public StateFlowBase, public Singleton<StatusDisplay>
{
public:
  StatusDisplay(openlcb::SimpleCanStack *, Service *);
  void clear();
  void replaceLine(int, const std::string&, ...);
private:
  STATE_FLOW_STATE(init);
  STATE_FLOW_STATE(i2cScan);
  STATE_FLOW_STATE(initOLED);
  STATE_FLOW_STATE(initLCD);
  STATE_FLOW_STATE(update);
  OSMutex lock_;
  std::string screenLines_[5]{"", "", "", "", ""};
  bool redraw_{true};
  StateFlowTimer timer_{this};
  std::unique_ptr<LCCStatCollector> lccStatCollector_;
  const uint8_t lineCount_;
  uint8_t rotatingIndex_{0};
  uint8_t updateCount_{0};
};

#endif // STATUS_DISPLAY_H_