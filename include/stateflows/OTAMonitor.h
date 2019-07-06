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

#pragma once

#include "stateflows/InfoScreen.h"
#include "interfaces/NextionInterface.h"

#include <executor/StateFlow.hxx>
#include <openlcb/SimpleStack.hxx>

class OTAMonitorFlow : public StateFlowBase {
public:
  OTAMonitorFlow(openlcb::SimpleCanStack *stack) : StateFlowBase(stack->service())
  {
#if NEXTION_ENABLED
    titlePage_ = static_cast<NextionTitlePage *>(nextionPages[TITLE_PAGE]);
#endif
  }

  void report_start()
  {
    progress_ = 0;

    // set blink pattern to alternating green blink
    statusLED->setStatusLED(StatusLED::LED::WIFI, StatusLED::COLOR::GREEN_BLINK, true);
    statusLED->setStatusLED(StatusLED::LED::OPS_TRACK, StatusLED::COLOR::GREEN_BLINK);
    statusLED->setStatusLED(StatusLED::LED::PROG_TRACK, StatusLED::COLOR::GREEN_BLINK, true);
    statusLED->setStatusLED(StatusLED::LED::EXT_1, StatusLED::COLOR::GREEN_BLINK);
    statusLED->setStatusLED(StatusLED::LED::EXT_2, StatusLED::COLOR::GREEN_BLINK, true);
    infoScreen->replaceLine(INFO_SCREEN_STATION_INFO_LINE, "Update starting");
#if NEXTION_ENABLED
    titlePage_->show();
    titlePage_->setStatusText(0, "OTA Started...");
#endif
  }

  void report_success()
  {
    // set blink pattern for all green
    statusLED->setStatusLED(StatusLED::LED::WIFI, StatusLED::COLOR::GREEN);
    statusLED->setStatusLED(StatusLED::LED::OPS_TRACK, StatusLED::COLOR::GREEN);
    statusLED->setStatusLED(StatusLED::LED::PROG_TRACK, StatusLED::COLOR::GREEN);
    statusLED->setStatusLED(StatusLED::LED::EXT_1, StatusLED::COLOR::GREEN);
    statusLED->setStatusLED(StatusLED::LED::EXT_2, StatusLED::COLOR::GREEN);
#if NEXTION_ENABLED
    titlePage_->setStatusText(1, "Update Complete");
    titlePage_->setStatusText(2, "Rebooting");
#endif
    infoScreen->replaceLine(INFO_SCREEN_STATION_INFO_LINE, "Update Complete");

    // reset countdown and trigger the restart
    countdown_ = StatusLED::LED::MAX_LED;
    start_flow(STATE(delay_and_reboot));
  }

  void report_failure(esp_err_t err)
  {
    // set blink pattern for alternating red blink
    statusLED->setStatusLED(StatusLED::LED::WIFI, StatusLED::COLOR::RED_BLINK, true);
    statusLED->setStatusLED(StatusLED::LED::OPS_TRACK, StatusLED::COLOR::RED_BLINK);
    statusLED->setStatusLED(StatusLED::LED::PROG_TRACK, StatusLED::COLOR::RED_BLINK, true);
    statusLED->setStatusLED(StatusLED::LED::EXT_1, StatusLED::COLOR::RED_BLINK);
    statusLED->setStatusLED(StatusLED::LED::EXT_2, StatusLED::COLOR::RED_BLINK, true);
#if NEXTION_ENABLED
    titlePage_->setStatusText(1, esp_err_to_name(err));
#endif
    infoScreen->replaceLine(INFO_SCREEN_STATION_INFO_LINE, esp_err_to_name(err));

    // restart the node due to failure
    start_flow(STATE(reboot_node));
   }

  void report_progress(uint32_t progress)
  {
    progress_ += progress;
    infoScreen->replaceLine(INFO_SCREEN_STATION_INFO_LINE, "Recv: %d", progress_);
#if NEXTION_ENABLED
    titlePage_->setStatusText(1, StringPrintf("Received: %d", progress_).c_str());
#endif
  }

private:
  StateFlowTimer timer_{this};
  uint8_t countdown_{StatusLED::LED::MAX_LED};
  uint32_t progress_{0};
#if NEXTION_ENABLED
  NextionTitlePage *titlePage_;
#endif

  Action reboot_node()
  {
    reboot();
    return exit();
  }

  Action delay_and_reboot()
  {
    if(countdown_ > 0)
    {
      // turn off lights
      statusLED->setStatusLED((StatusLED::LED)countdown_, StatusLED::COLOR::OFF);
      infoScreen->replaceLine(INFO_SCREEN_STATION_INFO_LINE, "reboot in %2d...", countdown_);
      LOG(WARNING, "ESP32 will reboot in %d seconds...", countdown_);
      --countdown_;
      return sleep_and_call(&timer_, SEC_TO_NSEC(1), STATE(reboot_node));
    }
    LOG(WARNING, "ESP32 will reboot NOW!");
    return call_immediately(STATE(reboot_node));
  }
};