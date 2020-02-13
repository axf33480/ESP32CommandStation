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
#include <utils/Uninitialized.hxx>
#include "stateflows/LCCStatCollector.h"

#if CONFIG_DISPLAY_TYPE_OLED
/// This is the width of the font used on the OLED display.
static constexpr uint8_t OLED_FONT_WIDTH = 8;

/// This is the height of the font used on the OLED display.
static constexpr uint8_t OLED_FONT_HEIGHT = 8;
#endif

class StatusDisplay : public StateFlowBase, public Singleton<StatusDisplay>
{
public:
  StatusDisplay(openlcb::SimpleCanStack *, Service *);
  void clear();
  void info(const std::string&, ...);
  void status(const std::string&, ...);
  void wifi(const std::string&, ...);
  void tcp_clients(const std::string&, ...);
  void track_power(const std::string&, ...);
private:
  STATE_FLOW_STATE(init);
  STATE_FLOW_STATE(initOLED);
  STATE_FLOW_STATE(initLCD);
  STATE_FLOW_STATE(update);

#if CONFIG_DISPLAY_TYPE_OLED
  /// Maximum number of characters that can be displayed in a single row on the
  /// OLED display.
  static constexpr uint8_t TEXT_COLUMN_COUNT =
    CONFIG_DISPLAY_OLED_WIDTH / OLED_FONT_WIDTH;

  /// Maximum number of rows that can be displayed on the OLED screen.
  static constexpr uint8_t TEXT_ROW_COUNT = CONFIG_DISPLAY_OLED_LINE_COUNT;
#elif CONFIG_DISPLAY_TYPE_LCD
  /// Maximum number of characters that can be displayed in a single row on the
  /// LCD display.
  static constexpr uint8_t TEXT_COLUMN_COUNT = 16;

  /// Maximum number of rows that can be displayed on the LCD screen.
  static constexpr uint8_t TEXT_ROW_COUNT = CONFIG_DISPLAY_LCD_LINE_COUNT;
#else
  /// Default value for when neither OLED or LCD are enabled.
  static constexpr uint8_t TEXT_ROW_COUNT = 1;
#endif

  /// Cache of the text to display on the OLED/LCD
  std::string screenLines_[TEXT_ROW_COUNT];

  bool redraw_{true};
  bool sh1106_{false};
  StateFlowTimer timer_{this};
  uint8_t regZero_{0};
  uninitialized<LCCStatCollector> lccStatCollector_;
  uint8_t rotatingIndex_{0};
  const uint8_t rotatingLineCount_;
  uint8_t updateCount_{0};
};

#endif // STATUS_DISPLAY_H_