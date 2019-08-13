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

#include "ESP32CommandStation.h"
#include <esp_event.h>

#include <freertos_drivers/arduino/WifiDefs.hxx>
#include <os/MDNS.hxx>
#include <utils/socket_listener.hxx>

void *jmriClientHandler(void *arg);

MDNS mDNS;
ESP32CSWebServer esp32csWebServer(&mDNS);
OSMutex jmriClientsMux;
vector<int> jmriClients;
unique_ptr<SocketListener> JMRIListener;

WiFiInterface wifiInterface;

constexpr int JMRI_CLIENT_PRIORITY = 1;
constexpr size_t JMRI_CLIENT_STACK_SIZE = 4096;
constexpr uint16_t JMRI_LISTENER_PORT = 2560;

WiFiInterface::WiFiInterface()
{
}

void WiFiInterface::init()
{
#if NEXTION_ENABLED
  auto nextionTitlePage = static_cast<NextionTitlePage *>(nextionPages[TITLE_PAGE]);
  nextionTitlePage->setStatusText(0, "Initializing WiFi");
#endif
  Singleton<InfoScreen>::instance()->replaceLine(
    INFO_SCREEN_IP_ADDR_LINE, "IP:Pending");
  Singleton<InfoScreen>::instance()->replaceLine(
    INFO_SCREEN_CLIENTS_LINE, "TCP Conn: 00");

  wifiManager->add_event_callback([](system_event_t *event) {
#if NEXTION_ENABLED
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
        wifiInterface.setIP(ip_info);
        Singleton<InfoScreen>::instance()->replaceLine(INFO_SCREEN_IP_ADDR_LINE
#if (INFO_SCREEN_LCD && INFO_SCREEN_LCD_COLUMNS >= 20) || INFO_SCREEN_OLED
                              , "IP: "
#endif
                              , IPSTR, IP2STR(&ip_info.ip)
        );
      }
      else
      {
        Singleton<StatusLED>::instance()->setStatusLED(
          StatusLED::LED::WIFI, StatusLED::COLOR::BLUE);
        Singleton<InfoScreen>::instance()->replaceLine(
          INFO_SCREEN_IP_ADDR_LINE, "SSID: %s"
        , configStore->getSSID().c_str());
      }
      if (!JMRIListener)
      {
        LOG(INFO, "[WiFi] Starting JMRI listener");
        JMRIListener.reset(new SocketListener(JMRI_LISTENER_PORT, [](int fd)
        {
          int clientCount = 0;
          {
            OSMutexLock h(&jmriClientsMux);
            jmriClients.push_back(fd);
            clientCount = webSocketClients.size() + jmriClients.size();
          }
          os_thread_create(nullptr, StringPrintf("jmri-%d", fd).c_str(),
                          JMRI_CLIENT_PRIORITY, JMRI_CLIENT_STACK_SIZE,
                          jmriClientHandler, (void *)fd);
          Singleton<InfoScreen>::instance()->replaceLine(
            INFO_SCREEN_CLIENTS_LINE, "TCP Conn: %02d", clientCount);
        }));
        mDNS.publish("jmri", "_esp32cs._tcp", JMRI_LISTENER_PORT);
        esp32csWebServer.begin();
      }
#if NEXTION_ENABLED
      nextionTitlePage->clearStatusText();
      // transition to next screen since WiFi connection is complete
      nextionPages[THROTTLE_PAGE]->display();
#endif
    } else if (event->event_id == SYSTEM_EVENT_STA_LOST_IP ||
               event->event_id == SYSTEM_EVENT_AP_STOP)
    {
      Singleton<StatusLED>::instance()->setStatusLED(
        StatusLED::LED::WIFI, StatusLED::COLOR::RED);
      LOG(INFO, "[WiFi] Shutting down JMRI listener");
      JMRIListener.reset(nullptr);
      Singleton<InfoScreen>::instance()->replaceLine(
        INFO_SCREEN_IP_ADDR_LINE, "Disconnected");
    }
    else if (event->event_id == SYSTEM_EVENT_STA_DISCONNECTED)
    {
      Singleton<StatusLED>::instance()->setStatusLED(
        StatusLED::LED::WIFI, StatusLED::COLOR::GREEN_BLINK);
    }
    else if (event->event_id == SYSTEM_EVENT_STA_START)
    {
      Singleton<StatusLED>::instance()->setStatusLED(
        StatusLED::LED::WIFI, StatusLED::COLOR::GREEN_BLINK);
#if NEXTION_ENABLED
      nextionTitlePage->setStatusText(0, "Connecting to WiFi");
#endif
    }
  });
}

string WiFiInterface::getStateAsDCCpp()
{
  return StringPrintf("<N1: " IPSTR " >", IP2STR(&ip_.ip));
}

void *jmriClientHandler(void *arg)
{
  int fd = (int)arg;
  unique_ptr<uint8_t> buf(new uint8_t[128]);
  HASSERT(buf.get() != nullptr);

  unique_ptr<DCCPPProtocolConsumer> consumer(new DCCPPProtocolConsumer());
  HASSERT(consumer.get() != nullptr);

  // tell JMRI about our state
  DCCPPProtocolHandler::process("s");

  while (true)
  {
    int bytesRead = ::read(fd, buf.get(), 128);
    if (bytesRead < 0 && (errno == EINTR || errno == EAGAIN))
    {
      // no data to read yet
    }
    else if (bytesRead > 0)
    {
      string res = consumer->feed(buf.get(), bytesRead);
      if (!res.empty())
      {
        ::write(fd, res.c_str(), res.length());
      }
    }
    else if (bytesRead == 0)
    {
      // EOF, close client
      LOG(INFO, "[JMRI %d] disconnected", fd);
      break;
    }
    else
    {
      // some other error, close client
      LOG(INFO, "[JMRI %d] error:%d, %s. Disconnecting.", fd, errno
        , strerror(errno));
      break;
    }
  }
  // remove client FD
  int clientCount = 0;
  {
    OSMutexLock h(&jmriClientsMux);
    jmriClients.erase(std::remove(jmriClients.begin(), jmriClients.end(), fd));
    clientCount = webSocketClients.size() + jmriClients.size();
  }
  Singleton<InfoScreen>::instance()->replaceLine(
    INFO_SCREEN_CLIENTS_LINE, "TCP Conn: %02d", clientCount);

  ::close(fd);
  return nullptr;
}
