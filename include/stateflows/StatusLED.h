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

#include "ESP32CommandStation.h"

#include <executor/StateFlow.hxx>
#include <openlcb/SimpleStack.hxx>
#include <NeoPixelBrightnessBus.h>

#define NEO_COLOR_TYPE RgbColor
#if STATUS_LED_COLOR_ORDER == RGB
#define NEO_COLOR_MODE NeoRgbFeature
#elif STATUS_LED_COLOR_ORDER == GRB
#define NEO_COLOR_MODE NeoGrbFeature
#elif STATUS_LED_COLOR_ORDER == RGBW
#define NEO_COLOR_MODE NeoRgbwFeature
#define NEO_COLOR_TYPE RgbwColor
#elif STATUS_LED_COLOR_ORDER == GRBW
#define NEO_COLOR_MODE NeoGrbwFeature
#define NEO_COLOR_TYPE RgbwColor
#elif STATUS_LED_COLOR_ORDER == BRG
#define NEO_COLOR_MODE NeoBrgFeature
#elif STATUS_LED_COLOR_ORDER == RBG
#define NEO_COLOR_MODE NeoRbgFeature
#else
#error "StatusLED: unknown LED color order"
#endif

#if STATUS_LED_TYPE == WS281X_800
#define NEO_METHOD Neo800KbpsMethod
#elif STATUS_LED_TYPE == WS281X_400
#define NEO_METHOD Neo400KbpsMethod
#elif STATUS_LED_TYPE == SK6812 || STATUS_LED_TYPE == LC6812
#define NEO_METHOD NeoSk6812Method
#else
#error "StatusLED: unknown LED type"
#endif

class StatusLED : public StateFlowBase {
public:
  enum COLOR {
    OFF,
    RED,
    GREEN,
    YELLOW,
    RED_BLINK,
    GREEN_BLINK,
    YELLOW_BLINK,
  };

  enum LED {
    WIFI,
    OPS_TRACK,
    PROG_TRACK,
    EXT_1,
    EXT_2,
    MAX_LED
  };

  StatusLED(openlcb::SimpleCanStack *stack) : StateFlowBase(stack->service()) {
    for(int index = 0; index < LED::MAX_LED; index++) {
      colors_[index] = RGB_OFF_;
      state_[index] = false;
    }
#if STATUS_LED_ENABLED
    start_flow(STATE(init));
#endif
  }

  void setStatusLED(const LED, const COLOR, const bool=false);
private:
  StateFlowTimer timer_{this};
  std::unique_ptr<NeoPixelBrightnessBus<NEO_COLOR_MODE, NEO_METHOD>> bus_{nullptr};
  NEO_COLOR_TYPE colors_[LED::MAX_LED];
  bool state_[LED::MAX_LED];
  const uint64_t updateInterval_{MSEC_TO_NSEC(450)};

  NEO_COLOR_TYPE RGB_RED_{NEO_COLOR_TYPE(255, 0, 0)};
  NEO_COLOR_TYPE RGB_GREEN_{NEO_COLOR_TYPE(0, 255, 0)};
  NEO_COLOR_TYPE RGB_YELLOW_{NEO_COLOR_TYPE(255, 255, 0)};
  NEO_COLOR_TYPE RGB_OFF_{NEO_COLOR_TYPE(0)};

  STATE_FLOW_STATE(init);
  STATE_FLOW_STATE(update);
};

extern unique_ptr<StatusLED> statusLED;