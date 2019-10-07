/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2018-2019 NormHal, Mike Dunston

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

#if NEXTION_UART_NUM == 2
Nextion nextion(Serial2);
#elif NEXTION_UART_NUM == 1
Nextion nextion(Serial1);
#else
#error "Invalid configuration detected for the NEXTION_UART_NUM value. Only UART 1 or UART 2 are supported for the Nextion Interface."
#endif

BaseNextionPage *nextionPages[MAX_PAGES] =
{
  new NextionTitlePage(nextion),
  new NextionAddressPage(nextion),
  new NextionThrottlePage(nextion),
  new NextionTurnoutPage(nextion),
  new NextionSetupPage(nextion),
  /* new NextionRoutesPage(nextion) */ nullptr
};

static constexpr char const * NEXTION_DISPLAY_TYPE_STRINGS[] =
{
  "basic 3.2\"",
  "basic 3.5\"",
  "basic 5.0\"",
  "enhanced 3.2\"",
  "enhanced 3.5\"",
  "enhanced 5.0\"",
  "Unknown"
};

NEXTION_DEVICE_TYPE nextionDeviceType{NEXTION_DEVICE_TYPE::UNKOWN_DISPLAY};

#if NEXTION_ENABLED
class NextionExecutable : public Executable
{
public:
  void run() override
  {
    if (!initialized_)
    {
      LOG(INFO, "[Nextion] Initializing UART(%d) at %ul baud on RX %d, TX %d"
        , NEXTION_UART_NUM, config_nextion_uart_speed()
        , config_nextion_rx_pin(), config_nextion_tx_pin());
#if NEXTION_UART_NUM == 2
      Serial2.begin(config_nextion_uart_speed(), SERIAL_8N1
                  , config_nextion_rx_pin(), config_nextion_tx_pin());
#elif NEXTION_UART_NUM == 1
      Serial1.begin(config_nextion_uart_speed(), SERIAL_8N1
                  , config_nextion_rx_pin(), config_nextion_tx_pin());
#endif
      initialized_ = nextion.init();
    }
    if (!screenDetected_)
    {
      NextionTitlePage * titlePage =
        static_cast<NextionTitlePage *>(nextionPages[TITLE_PAGE]);
      // attempt to identify the nextion display.
      constexpr uint8_t MAX_ATTEMPTS = 3;
      uint8_t attempt = 0;
      while(attempt++ < MAX_ATTEMPTS &&
            nextionDeviceType == NEXTION_DEVICE_TYPE::UNKOWN_DISPLAY)
      {
        LOG(INFO
          , "[Nextion] [%d/%d] Attempting to identify the attached Nextion display"
          , attempt, MAX_ATTEMPTS);
        nextion.sendCommand("DRAKJHSUYDGBNCJHGJKSHBDN");
        nextion.sendCommand("connect");
        String screenID = "";
        size_t res = nextion.receiveString(screenID, false);
        if(res && screenID.indexOf("comok") >= 0) {
          // break the returned string into its comma delimited chunks
          // start after the first space
          std::stringstream buf(screenID.substring(screenID.indexOf(' ') + 1).c_str());
          vector<string> parts;
          string part;
          while(getline(buf, part, ',')) {
            parts.push_back(part);
          }

          // attempt to parse device model
          if(!parts[2].compare(0, 7, "NX4024K"))
          {
            nextionDeviceType = NEXTION_DEVICE_TYPE::ENHANCED_3_2_DISPLAY;
          }
          else if(!parts[2].compare(0, 7, "NX4024T"))
          {
            nextionDeviceType = NEXTION_DEVICE_TYPE::BASIC_3_2_DISPLAY;
          }
          else if(!parts[2].compare(0, 7, "NX4832K"))
          {
            nextionDeviceType = NEXTION_DEVICE_TYPE::ENHANCED_3_5_DISPLAY;
          }
          else if(!parts[2].compare(0, 7, "NX4832T"))
          {
            nextionDeviceType = NEXTION_DEVICE_TYPE::BASIC_3_5_DISPLAY;
          }
          else if(!parts[2].compare(0, 7, "NX8048K"))
          {
            nextionDeviceType = NEXTION_DEVICE_TYPE::ENHANCED_5_0_DISPLAY;
          }
          else if(!parts[2].compare(0, 7, "NX8048T"))
          {
            nextionDeviceType = NEXTION_DEVICE_TYPE::BASIC_5_0_DISPLAY;
          }
          else
          {
            LOG(WARNING, "[Nextion] Unrecognized Nextion Device model: %s"
              , parts[2].c_str());
          }
          LOG(INFO, "[Nextion] Device type: %s"
            , NEXTION_DISPLAY_TYPE_STRINGS[nextionDeviceType]);
          LOG(INFO, "[Nextion] Firmware Version: %s", parts[3].c_str());
          LOG(INFO, "[Nextion] MCU Code: %s", parts[4].c_str());
          LOG(INFO, "[Nextion] Serial #: %s", parts[5].c_str());
          LOG(INFO, "[Nextion] Flash size: %s bytes", parts[6].c_str());
        }
        else
        {
          LOG(WARNING
            , "[Nextion] Unable to determine Nextion device type: %s"
            , screenID.c_str());
        }
      }
      if(nextionDeviceType == NEXTION_DEVICE_TYPE::UNKOWN_DISPLAY)
      {
        LOG(WARNING
          , "[Nextion] Failed to identify the attached Nextion display, "
            "defaulting to 3.2\" basic display");
        nextionDeviceType = NEXTION_DEVICE_TYPE::BASIC_3_2_DISPLAY;
      }

      // flush the serial buffer after detection
#if NEXTION_UART_NUM == 2
      Serial2.flush();
#elif NEXTION_UART_NUM == 1
      Serial1.flush();
#endif

      titlePage->display();
      titlePage->setStatusText(3, "Detected Screen type:");
      titlePage->setStatusText(4, NEXTION_DISPLAY_TYPE_STRINGS[nextionDeviceType]);
      screenDetected_ = true;
    }

    nextion.poll();
  }
private:
  bool initialized_{false};
  bool screenDetected_{false};
};
#endif // NEXTION_ENABLED

void nextionInterfaceInit()
{
#if NEXTION_ENABLED
  Singleton<InfoScreen>::instance()->replaceLine(
    INFO_SCREEN_ROTATING_STATUS_LINE, "Init Nextion");
  extern unique_ptr<OpenMRN> openmrn;
  openmrn->stack()->executor()->add(new NextionExecutable());
#endif // NEXTION_ENABLED
}

