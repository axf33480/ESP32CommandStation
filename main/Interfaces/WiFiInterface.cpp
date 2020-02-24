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
#include <utils/socket_listener.hxx>

void *jmriClientHandler(void *arg);

MDNS mDNS;
OSMutex jmriClientsMux;
vector<int> jmriClients;
unique_ptr<SocketListener> JMRIListener;

constexpr uint16_t JMRI_LISTENER_PORT = 2560;

class JmriClient : private StateFlowBase, public DCCPPProtocolConsumer
{
public:
  JmriClient(int fd, uint32_t remote_ip, Service *service)
    : StateFlowBase(service), DCCPPProtocolConsumer(), fd_(fd), remoteIP_(remote_ip)
  {
    LOG(INFO, "[JMRI %s] Connected", name().c_str());
    {
      OSMutexLock h(&jmriClientsMux);
      jmriClients.push_back(fd_);
    }
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
    {
      OSMutexLock h(&jmriClientsMux);
      jmriClients.erase(std::remove(jmriClients.begin(), jmriClients.end()
                      , fd_));
    }
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

void init_webserver(MDNS *dns);

void init_wifi_endpoints()
{
  init_webserver(&mDNS);
#if CONFIG_NEXTION
  auto nextionTitlePage = static_cast<NextionTitlePage *>(nextionPages[TITLE_PAGE]);
  nextionTitlePage->setStatusText(0, "Initializing WiFi");
#endif
  Singleton<StatusDisplay>::instance()->wifi("IP:Pending");

  Singleton<Esp32WiFiManager>::instance()->add_event_callback([](system_event_t *event) {
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
                       , Singleton<http::Httpd>::instance());
        }));
        mDNS.publish("jmri", "_esp32cs._tcp", JMRI_LISTENER_PORT);
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
      LOG(INFO, "[WiFi] Shutting down JMRI listener");
      JMRIListener.reset(nullptr);
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
