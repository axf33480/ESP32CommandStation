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

#include "ESP32CommandStation.h"
#include <esp_event.h>

#include <freertos_drivers/arduino/WifiDefs.hxx>
#include <Httpd.h>
#include <os/MDNS.hxx>
#include <StatusDisplay.h>


MDNS mDNS;

void init_webserver(MDNS *dns);

void init_wifi_endpoints()
{
  init_webserver(&mDNS);
#if CONFIG_NEXTION
  auto nextionTitlePage = static_cast<NextionTitlePage *>(nextionPages[TITLE_PAGE]);
  nextionTitlePage->setStatusText(0, "Initializing WiFi");
#endif

  Singleton<Esp32WiFiManager>::instance()->add_event_callback(
  [](system_event_t *event)
  {
#if CONFIG_NEXTION
    auto nextionTitlePage =
      static_cast<NextionTitlePage *>(nextionPages[TITLE_PAGE]);
#endif
    if(event->event_id == SYSTEM_EVENT_STA_GOT_IP ||
       event->event_id == SYSTEM_EVENT_AP_START)
    {
      if (event->event_id == SYSTEM_EVENT_STA_GOT_IP)
      {
        Singleton<StatusLED>::instance()->setStatusLED(
          StatusLED::LED::WIFI, StatusLED::COLOR::GREEN);
        tcpip_adapter_ip_info_t ip_info;
        tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info);
        Singleton<StatusDisplay>::instance()->wifi(
#if CONFIG_DISPLAY_COLUMN_COUNT > 16 || CONFIG_DISPLAY_TYPE_OLED
                              "IP: "
#endif
                              IPSTR, IP2STR(&ip_info.ip)
        );
      }
      else
      {
        Singleton<StatusLED>::instance()->setStatusLED(
          StatusLED::LED::WIFI, StatusLED::COLOR::BLUE);
        Singleton<StatusDisplay>::instance()->wifi("SSID: %s"
        , Singleton<ConfigurationManager>::instance()->getSSID().c_str());
      }
#if CONFIG_NEXTION
      nextionTitlePage->clearStatusText();
      // transition to next screen since WiFi connection is complete
      nextionPages[THROTTLE_PAGE]->display();
#endif
    } else if (event->event_id == SYSTEM_EVENT_STA_LOST_IP ||
               event->event_id == SYSTEM_EVENT_AP_STOP)
    {
      Singleton<StatusLED>::instance()->setStatusLED(
        StatusLED::LED::WIFI, StatusLED::COLOR::RED);
      Singleton<StatusDisplay>::instance()->wifi("Disconnected");
    }
    else if (event->event_id == SYSTEM_EVENT_STA_DISCONNECTED ||
             event->event_id == SYSTEM_EVENT_STA_START)
    {
      Singleton<StatusLED>::instance()->setStatusLED(
        StatusLED::LED::WIFI, StatusLED::COLOR::GREEN_BLINK);
#if CONFIG_NEXTION
      nextionTitlePage->setStatusText(0, "Connecting to WiFi");
#endif
    }
  });
}
