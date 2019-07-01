/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2019 Mike Dunston

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

StateFlowBase::Action StatusLED::init() {
  LOG(INFO, "[StatusLED] Initializing LEDs");
#if defined(STATUS_LED_DATA_PIN) && defined(STATUS_LED_BRIGHTNESS)
  bus_.reset(new NeoPixelBrightnessBus<NEO_COLOR_MODE, NEO_METHOD>(LED::MAX_LED, STATUS_LED_DATA_PIN));
  bus_->Begin();
  bus_->SetBrightness(STATUS_LED_BRIGHTNESS);
  bus_->ClearTo(RGB_OFF_);
  bus_->Show();
  return sleep_and_call(&timer_, updateInterval_, STATE(update));
#endif
  return exit();
}

StateFlowBase::Action StatusLED::update() {
  for(int led = 0; led < LED::MAX_LED; led++) {
    // if the LED is set to blink, toggle it
    if(colors_[led] == RED_BLINK || colors_[led] == GREEN_BLINK || colors_[led] == YELLOW_BLINK) {
      if(state_[led]) {
        bus_->SetPixelColor(led, RGB_OFF_);
      } else if(colors_[led] == RED_BLINK) {
        bus_->SetPixelColor(led, RGB_RED_);
      } else if(colors_[led] == GREEN_BLINK) {
        bus_->SetPixelColor(led, RGB_GREEN_);
      } else if(colors_[led] == YELLOW_BLINK) {
        bus_->SetPixelColor(led, RGB_YELLOW_);
      }
      state_[led] = !state_[led];
    }
  }
  bus_->Show();
  return sleep_and_call(&timer_, updateInterval_, STATE(update));
}

void StatusLED::setStatusLED(const LED led, const COLOR color, const bool on) {
  colors_[led] = color;
  state_[led] = on;
}

