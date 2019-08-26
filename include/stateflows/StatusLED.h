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

#ifndef STATUS_LED_H_
#define STATUS_LED_H_

#include <executor/Service.hxx>
#include <executor/StateFlow.hxx>
#include <NeoPixelBrightnessBus.h>

#include "ESP32CSConstants.h"

// constants for pre-compiler checks
#define RGB   1
#define GRB   2
#define RGBW  3
#define GRBW  4
#define BRG   5
#define RBG   6

#if STATUS_LED_COLOR_ORDER == RGB
#define NEO_COLOR_TYPE RgbColor
#define NEO_COLOR_MODE NeoRgbFeature
#define NEO_COLOR_MODE_NAME "RGB"
#elif STATUS_LED_COLOR_ORDER == GRB
#define NEO_COLOR_MODE NeoGrbFeature
#define NEO_COLOR_MODE_NAME "GRB"
#elif STATUS_LED_COLOR_ORDER == RGBW
#define NEO_COLOR_MODE NeoRgbwFeature
#define NEO_COLOR_TYPE RgbwColor
#define NEO_COLOR_MODE_NAME "RGBW"
#elif STATUS_LED_COLOR_ORDER == GRBW
#define NEO_COLOR_MODE NeoGrbwFeature
#define NEO_COLOR_TYPE RgbwColor
#define NEO_COLOR_MODE_NAME "GRBW"
#elif STATUS_LED_COLOR_ORDER == BRG
#define NEO_COLOR_TYPE RgbColor
#define NEO_COLOR_MODE NeoBrgFeature
#define NEO_COLOR_MODE_NAME "BRG"
#elif STATUS_LED_COLOR_ORDER == RBG
#define NEO_COLOR_TYPE RgbColor
#define NEO_COLOR_MODE NeoRbgFeature
#define NEO_COLOR_MODE_NAME "RBG"
#else
#error "StatusLED: unknown LED color order"
#endif

#define WS281X      1
#define WS281X_800K 2
#define WS281X_400K 3
#define SK6812      4
#define LC6812      5
#define APA106      6

#if STATUS_LED_TYPE == WS281X
#define NEO_METHOD NeoEsp32Rmt6Ws2812xMethod
#define NEO_METHOD_NAME "RMT(6)-Ws2812"
#elif STATUS_LED_TYPE == WS281X_800K
#define NEO_METHOD NeoEsp32Rmt6800KbpsMethod
#define NEO_METHOD_NAME "RMT(6)-Ws2812-800kbps"
#elif STATUS_LED_TYPE == WS281X_400K
#define NEO_METHOD NeoEsp32Rmt6400KbpsMethod
#define NEO_METHOD_NAME "RMT(6)-Ws2812-400kbps"
#elif STATUS_LED_TYPE == SK6812 || STATUS_LED_TYPE == LC6812
#define NEO_METHOD NeoEsp32Rmt6Sk6812Method
#define NEO_METHOD_NAME "RMT(6)-sk6812"
#elif STATUS_LED_TYPE == APA106
#define NEO_METHOD NeoEsp32Rmt6Apa106Method
#define NEO_METHOD_NAME "RMT(6)-APA106"
#else
#error "StatusLED: unknown LED type"
#endif

class StatusLED : public StateFlowBase, public Singleton<StatusLED>
{
public:
  enum COLOR : uint8_t
  {
    OFF
  , RED
  , GREEN
  , YELLOW
  , BLUE
  , RED_BLINK
  , GREEN_BLINK
  , BLUE_BLINK
  , YELLOW_BLINK
  };

  enum LED : uint8_t
  {
    WIFI,
    OPS_TRACK,
    PROG_TRACK,
    EXT_1,
    EXT_2,
    MAX_LED
  };

  StatusLED(Service *service)
    : StateFlowBase(service)
    , bus_(nullptr)
    , updateInterval_(MSEC_TO_NSEC(config_status_led_update_interval_msec()))
  {
    for(int index = 0; index < LED::MAX_LED; index++)
    {
      colors_[index] = RGB_OFF_;
      state_[index] = false;
    }
    if (config_status_led_enabled() == CONSTANT_TRUE)
    {
      start_flow(STATE(init));
    }
  }

  void setStatusLED(const LED, const COLOR, const bool=false);
private:
  StateFlowTimer timer_{this};
  std::unique_ptr<NeoPixelBrightnessBus<NEO_COLOR_MODE, NEO_METHOD>> bus_;
  NEO_COLOR_TYPE colors_[LED::MAX_LED];
  bool state_[LED::MAX_LED];
  const uint64_t updateInterval_;

  NEO_COLOR_TYPE RGB_RED_{NEO_COLOR_TYPE(255, 0, 0)};
  NEO_COLOR_TYPE RGB_GREEN_{NEO_COLOR_TYPE(0, 255, 0)};
  NEO_COLOR_TYPE RGB_YELLOW_{NEO_COLOR_TYPE(255, 255, 0)};
  NEO_COLOR_TYPE RGB_BLUE_{NEO_COLOR_TYPE(0, 0, 255)};
  NEO_COLOR_TYPE RGB_OFF_{NEO_COLOR_TYPE(0)};

  STATE_FLOW_STATE(init);
  STATE_FLOW_STATE(update);
  STATE_FLOW_STATE(update_bus);
};

#endif // STATUS_LED_H_