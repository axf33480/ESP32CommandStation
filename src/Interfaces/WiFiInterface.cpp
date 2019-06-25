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
#include <ESPAsyncWebServer.h>
#include "WebServer.h"

#include <freertos_drivers/arduino/WifiDefs.hxx>
#include <os/MDNS.hxx>
#include <utils/socket_listener.hxx>

#if HC12_RADIO_ENABLED
#include "HC12Interface.h"
#endif

void *jmriClientHandler(void *arg);

MDNS mDNS;
ESP32CSWebServer esp32csWebServer(&mDNS);
std::vector<int> jmriClients;
std::unique_ptr<SocketListener> JMRIListener;
WiFiInterface wifiInterface;
extern Esp32WiFiManager wifi_mgr;

constexpr int JMRI_CLIENT_PRIORITY = 0;
constexpr size_t JMRI_CLIENT_STACK_SIZE = 4096;
constexpr uint16_t JMRI_LISTENER_PORT = 2560;

WiFiInterface::WiFiInterface() {
}

void WiFiInterface::init() {
#if NEXTION_ENABLED
  auto nextionTitlePage = static_cast<NextionTitlePage *>(nextionPages[TITLE_PAGE]);
  nextionTitlePage->setStatusText(0, "Initializing WiFi");
#endif

  InfoScreen::replaceLine(INFO_SCREEN_ROTATING_STATUS_LINE, F("Init WiFI"));
  InfoScreen::replaceLine(INFO_SCREEN_IP_ADDR_LINE, F("IP:Pending"));
/*
#if defined(WIFI_STATIC_IP_ADDRESS) && defined(WIFI_STATIC_IP_GATEWAY) && defined(WIFI_STATIC_IP_SUBNET)
  IPAddress staticIP, gatewayIP, subnetMask, dnsServer;
  staticIP.fromString(WIFI_STATIC_IP_ADDRESS);
  gatewayIP.fromString(WIFI_STATIC_IP_GATEWAY);
  subnetMask.fromString(WIFI_STATIC_IP_SUBNET);
#if defined(WIFI_STATIC_IP_DNS)
  dnsServer.fromString(WIFI_STATIC_IP_DNS);
#else
  dnsServer.fromString("8.8.8.8");
#endif
  WiFi.config(staticIP, gatewayIP, subnetMask, dnsServer);
#endif
*/
  wifi_mgr.add_event_callback([](system_event_t *event) {
    if(event->event_id == SYSTEM_EVENT_STA_GOT_IP) {
#if STATUS_LED_ENABLED
      setStatusLED(STATUS_LED::WIFI_LED, STATUS_LED_COLOR::LED_GREEN);
#endif
      tcpip_adapter_ip_info_t ip_info;
      tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info);
      wifiInterface.setIP(ip_info);
#if INFO_SCREEN_ENABLED
  #if INFO_SCREEN_LCD && INFO_SCREEN_LCD_COLUMNS < 20
      InfoScreen::replaceLine(INFO_SCREEN_IP_ADDR_LINE, F(IPSTR), IP2STR(&ip_info.ip));
  #else
      InfoScreen::replaceLine(INFO_SCREEN_IP_ADDR_LINE, F("IP: " IPSTR), IP2STR(&ip_info.ip));
  #endif
#endif
      JMRIListener.reset(new SocketListener(JMRI_LISTENER_PORT, [](int fd) {
        jmriClients.push_back(fd);
        os_thread_create(nullptr, StringPrintf("jmri-%d", fd).c_str(),
                        JMRI_CLIENT_PRIORITY, JMRI_CLIENT_STACK_SIZE,
                        jmriClientHandler, (void *)fd);
      }));
      esp32csWebServer.begin();
      mDNS.publish("jmri", "_esp32cs._tcp", JMRI_LISTENER_PORT);
#if NEXTION_ENABLED
      static_cast<NextionTitlePage *>(nextionPages[TITLE_PAGE])->clearStatusText();
      // transition to next screen since WiFi connection is complete
      nextionPages[THROTTLE_PAGE]->display();
#endif
    } else if (event->event_id == SYSTEM_EVENT_STA_LOST_IP) {
#if STATUS_LED_ENABLED
      setStatusLED(STATUS_LED::WIFI_LED, STATUS_LED_COLOR::LED_RED);
#endif
      JMRIListener.reset(nullptr);
#if INFO_SCREEN_ENABLED
  #if INFO_SCREEN_LCD && INFO_SCREEN_LCD_COLUMNS < 20
      InfoScreen::replaceLine(INFO_SCREEN_IP_ADDR_LINE, "Disconnected");
  #else
      InfoScreen::print(3, INFO_SCREEN_IP_ADDR_LINE, "Disconnected");
  #endif
#endif
    } else if (event->event_id == SYSTEM_EVENT_STA_DISCONNECTED) {
#if STATUS_LED_ENABLED
      setStatusLED(STATUS_LED::WIFI_LED, STATUS_LED_COLOR::LED_RED);
#endif
    } else if (event->event_id == SYSTEM_EVENT_STA_START) {
#if STATUS_LED_ENABLED
      setStatusLED(STATUS_LED::WIFI_LED, STATUS_LED_COLOR::LED_GREEN_BLINK);
#endif
#if NEXTION_ENABLED
      nextionTitlePage->setStatusText(0, "Connecting to WiFi");
#endif
    }
  });
}

void WiFiInterface::showInitInfo() {
  print(F("<N1: " IPSTR " >"), IP2STR(&_ip_info.ip));
}

void WiFiInterface::send(const String &buf) {
  for (const int client : jmriClients) {
    ::write(client, buf.c_str(), buf.length());
  }
  esp32csWebServer.broadcastToWS(buf);
#if HC12_RADIO_ENABLED
  HC12Interface::send(buf);
#endif
}

void WiFiInterface::print(const __FlashStringHelper *fmt, ...) {
  char buf[256] = {0};
  va_list args;
  va_start(args, fmt);
  vsnprintf_P(buf, sizeof(buf), (const char *)fmt, args);
  va_end(args);
  send(buf);
}

void *jmriClientHandler(void *arg) {
  int fd = (int)arg;
  DCCPPProtocolConsumer consumer;
  std::unique_ptr<uint8_t> buf(new uint8_t[128]);
  HASSERT(buf.get() != nullptr);

  // tell JMRI about our state
  DCCPPProtocolHandler::process("s");

  while (true) {
    int bytesRead = ::read(fd, buf.get(), 128);
    if (bytesRead < 0 && (errno == EINTR || errno == EAGAIN)) {
      // no data to read yet
    } else if (bytesRead > 0) {
      consumer.feed(buf.get(), bytesRead);
    } else if (bytesRead == 0) {
      // EOF, close client
      LOG(INFO, "[JMRI %d] disconnected", fd);
      break;
    } else {
      // some other error, close client
      LOG(INFO, "[JMRI %d] error:%d, %s. Disconnecting.", fd, errno, strerror(errno));
      break;
    }
  }
  // remove client FD
  std::vector<int>::iterator it = std::find(jmriClients.begin(), jmriClients.end(), fd);
  if (it != jmriClients.end()) {
    jmriClients.erase(it);
  }
  ::close(fd);
  return nullptr;
}