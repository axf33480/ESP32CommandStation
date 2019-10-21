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
OSMutex jmriClientsMux;
vector<int> jmriClients;
unique_ptr<SocketListener> JMRIListener;

WiFiInterface wifiInterface;

constexpr uint16_t JMRI_LISTENER_PORT = 2560;

class JmriClient : private StateFlowBase, public DCCPPProtocolConsumer
{
public:
  JmriClient(int fd, uint32_t remote_ip, Service *service)
    : StateFlowBase(service), DCCPPProtocolConsumer(), fd_(fd), remoteIP_(remote_ip)
  {
    LOG(INFO, "[JMRI %s] Connected", name().c_str());
    int clientCount = 0;
    {
      OSMutexLock h(&jmriClientsMux);
      jmriClients.push_back(fd_);
      clientCount = webSocketClients.size() + jmriClients.size();
    }
    Singleton<InfoScreen>::instance()->replaceLine(
      INFO_SCREEN_CLIENTS_LINE, "TCP Conn: %02d", clientCount);
    bzero(buf_, BUFFER_SIZE);

    struct timeval tm;
    tm.tv_sec = 0;
    tm.tv_usec = MSEC_TO_USEC(10);
    ERRNOCHECK("setsockopt_timeout",
        setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tm, sizeof(tm)));

    start_flow(STATE(read_data));
  }

  virtual ~JmriClient()
  {
    LOG(INFO, "[JMRI %s] Disconnected", name().c_str());
    int clientCount = 0;
    {
      OSMutexLock h(&jmriClientsMux);
      jmriClients.erase(std::remove(jmriClients.begin(), jmriClients.end()
                      , fd_));
      clientCount = webSocketClients.size() + jmriClients.size();
    }
    Singleton<InfoScreen>::instance()->replaceLine(
      INFO_SCREEN_CLIENTS_LINE, "TCP Conn: %02d", clientCount);
    ::close(fd_);
  }
private:
  static const size_t BUFFER_SIZE = 128;
  int fd_;
  uint32_t remoteIP_;
  uint8_t buf_[BUFFER_SIZE];
  size_t buf_used_{0};
  string res_;
  StateFlowTimedSelectHelper helper_{this};

  STATE_FLOW_STATE(read_data);
  STATE_FLOW_STATE(process_data);
  STATE_FLOW_STATE(send_data);

  string name()
  {
    return StringPrintf("%s/%d", ipv4_to_string(remoteIP_).c_str(), fd_);
  }
};

StateFlowBase::Action JmriClient::read_data()
{
  // clear the buffer of data we have sent back
  res_.clear();

  return read_nonblocking(&helper_, fd_, buf_, BUFFER_SIZE
                        , STATE(process_data));
}

StateFlowBase::Action JmriClient::process_data()
{
  if (helper_.hasError_)
  {
    return delete_this();
  }
  else if (helper_.remaining_ == BUFFER_SIZE)
  {
    return yield_and_call(STATE(read_data));
  }
  else
  {
    buf_used_ = BUFFER_SIZE - helper_.remaining_;
    LOG(VERBOSE, "[JMRI %s] received %zu bytes", name().c_str(), buf_used_);
  }
  res_.append(feed(buf_, buf_used_));
  buf_used_ = 0;
  return yield_and_call(STATE(send_data));
}

StateFlowBase::Action JmriClient::send_data()
{
  if(res_.empty())
  {
    return yield_and_call(STATE(read_data));
  }
  return write_repeated(&helper_, fd_, res_.data(), res_.length()
                      , STATE(read_data));
}

WiFiInterface::WiFiInterface()
{
}

void WiFiInterface::init()
{
  init_webserver(&mDNS);
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
        Singleton<InfoScreen>::instance()->replaceLine(INFO_SCREEN_IP_ADDR_LINE
#if (INFO_SCREEN_LCD && INFO_SCREEN_LCD_COLUMNS >= 20) || INFO_SCREEN_OLED
                              , "IP: " IPSTR
#else
                              , IPSTR
#endif
                              , IP2STR(&ip_info.ip)
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
          sockaddr_in source;
          socklen_t source_len = sizeof(sockaddr_in);
          bzero(&source, sizeof(sockaddr_in));
          getpeername(fd, (sockaddr *)&source, &source_len);
          new JmriClient(fd, ntohl(source.sin_addr.s_addr)
                       , Singleton<esp32cs::http::Httpd>::instance());
        }));
        mDNS.publish("jmri", "_esp32cs._tcp", JMRI_LISTENER_PORT);
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
    else if (event->event_id == SYSTEM_EVENT_STA_DISCONNECTED ||
             event->event_id == SYSTEM_EVENT_STA_START)
    {
      Singleton<StatusLED>::instance()->setStatusLED(
        StatusLED::LED::WIFI, StatusLED::COLOR::GREEN_BLINK);
#if NEXTION_ENABLED
      nextionTitlePage->setStatusText(0, "Connecting to WiFi");
#endif
    }
  });
}
