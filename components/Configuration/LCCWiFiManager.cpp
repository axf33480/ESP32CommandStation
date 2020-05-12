/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2020 Mike Dunston

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

#include "LCCWiFiManager.h"
#include "JsonConstants.h"

#include "sdkconfig.h"

#include <algorithm>
#include <ConfigurationManager.h>
#include <freertos_drivers/esp32/Esp32WiFiManager.hxx>
#include <HttpStringUtils.h>
#include <openlcb/SimpleStack.hxx>

namespace esp32cs
{

static constexpr const char WIFI_STATION_CFG[] = "wifi-sta";
static constexpr const char WIFI_STATION_DNS_CFG[] = "wifi-dns";
static constexpr const char WIFI_STATION_IP_CFG[] = "wifi-ip";
static constexpr const char WIFI_SOFTAP_CFG[] = "wifi-ap";

#ifndef CONFIG_WIFI_STATIC_IP_ADDRESS
#define CONFIG_WIFI_STATIC_IP_ADDRESS ""
#endif

#ifndef CONFIG_WIFI_STATIC_IP_GATEWAY
#define CONFIG_WIFI_STATIC_IP_GATEWAY ""
#endif

#ifndef CONFIG_WIFI_STATIC_IP_SUBNET
#define CONFIG_WIFI_STATIC_IP_SUBNET ""
#endif

#ifndef CONFIG_WIFI_STATIC_IP_DNS
#define CONFIG_WIFI_STATIC_IP_DNS ""
#endif

#ifndef CONFIG_WIFI_SOFTAP_SSID
#define CONFIG_WIFI_SOFTAP_SSID "esp32cs"
#endif

#ifndef CONFIG_WIFI_SOFTAP_PASSWORD
#define CONFIG_WIFI_SOFTAP_PASSWORD "esp32cs"
#endif

#ifndef CONFIG_WIFI_SSID
#define CONFIG_WIFI_SSID "esp32cs"
#endif

#ifndef CONFIG_WIFI_PASSWORD
#define CONFIG_WIFI_PASSWORD "esp32cs"
#endif

LCCWiFiManager::LCCWiFiManager(openlcb::SimpleStackBase *stack
                             , const esp32cs::Esp32ConfigDef &cfg)
                             : stack_(stack), cfg_(cfg)
{
  auto cfg_mgr = Singleton<ConfigurationManager>::instance();
  if (!cfg_mgr->exists(WIFI_SOFTAP_CFG) && !cfg_mgr->exists(WIFI_STATION_CFG))
  {
#if defined(CONFIG_WIFI_MODE_SOFTAP)
    reconfigure_mode(JSON_VALUE_WIFI_MODE_SOFTAP_ONLY, false);
#elif defined(CONFIG_WIFI_MODE_SOFTAP_STATION)
    reconfigure_mode(JSON_VALUE_WIFI_MODE_SOFTAP_STATION, false);
#else
    reconfigure_mode(JSON_VALUE_WIFI_MODE_STATION_ONLY, false);
#endif
#if defined(CONFIG_WIFI_MODE_SOFTAP_STATION) || \
    defined(CONFIG_WIFI_MODE_STATION)
    reconfigure_station(CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD
                      , CONFIG_WIFI_STATIC_IP_ADDRESS
                      , CONFIG_WIFI_STATIC_IP_GATEWAY
                      , CONFIG_WIFI_STATIC_IP_SUBNET
                      , CONFIG_WIFI_STATIC_IP_DNS, false);
#endif // CONFIG_WIFI_MODE_SOFTAP_STATION || CONFIG_WIFI_MODE_STATION
  }

  LOG(INFO, "[WiFi] Loading configuration");
  if (cfg_mgr->exists(WIFI_SOFTAP_CFG) && !cfg_mgr->exists(WIFI_STATION_CFG))
  {
    mode_ =  WIFI_MODE_AP;
    string station_cfg = cfg_mgr->load(WIFI_SOFTAP_CFG);
    std::pair<string, string> cfg = http::break_string(station_cfg, "\n");
    ssid_ = cfg.first;
    password_ = cfg.second;
    LOG(INFO, "[WiFi] SoftAP only (ssid: %s)", ssid_.c_str());
  }
  else if (cfg_mgr->exists(WIFI_SOFTAP_CFG) &&
           cfg_mgr->exists(WIFI_STATION_CFG))
  {
    LOG(INFO, "[WiFi] SoftAP and Station");
    mode_ =  WIFI_MODE_APSTA;
  }
  else if (cfg_mgr->exists(WIFI_STATION_CFG))
  {
    LOG(INFO, "[WiFi] Station only");
    mode_ =  WIFI_MODE_STA;
  }
  else
  {
    LOG(INFO
      , "[WiFi] Unable to locate SoftAP or Station configuration, enabling "
        "SoftAP: %s", CONFIG_WIFI_SOFTAP_SSID);
    mode_ =  WIFI_MODE_AP;
    ssid_ = CONFIG_WIFI_SOFTAP_SSID;
    password_ = CONFIG_WIFI_SOFTAP_PASSWORD;
  }
  if (mode_ != WIFI_MODE_AP)
  {
    string station_cfg = cfg_mgr->load(WIFI_STATION_CFG);
    std::pair<string, string> cfg = http::break_string(station_cfg, "\n");
    ssid_ = cfg.first;
    password_ = cfg.second;
    if (cfg_mgr->exists(WIFI_STATION_IP_CFG))
    {
      std::vector<string> ip_parts;
      string ip_cfg = cfg_mgr->load(WIFI_STATION_IP_CFG);
      http::tokenize(ip_cfg, ip_parts, "\n");
      stationIP_.reset(new tcpip_adapter_ip_info_t());
      stationIP_->ip.addr = ipaddr_addr(ip_parts[0].c_str());
      stationIP_->gw.addr = ipaddr_addr(ip_parts[1].c_str());
      stationIP_->netmask.addr = ipaddr_addr(ip_parts[2].c_str());
      LOG(INFO, "[WiFi] Static IP:" IPSTR ", gateway:" IPSTR ",netmask:" IPSTR,
        IP2STR(&stationIP_->ip), IP2STR(&stationIP_->gw), IP2STR(&stationIP_->netmask));
    }
    if (cfg_mgr->exists(WIFI_STATION_DNS_CFG))
    {
      string dns = cfg_mgr->load(WIFI_STATION_DNS_CFG);
      stationDNS_.u_addr.ip4.addr = ipaddr_addr(dns.c_str());
      LOG(INFO, "[WiFi] DNS configured: " IPSTR
        , IP2STR(&stationDNS_.u_addr.ip4));
    }
  }
  else if (cfg_mgr->exists(WIFI_SOFTAP_CFG))
  {
    string station_cfg = cfg_mgr->load(WIFI_SOFTAP_CFG);
    std::pair<string, string> cfg = http::break_string(station_cfg, "\n");
    ssid_ = cfg.first;
    password_ = cfg.second;
  }

  // TODO: Switch to SimpleStackBase * instead of casting to SimpleCanStack *
  // once Esp32WiFiManager supports this.
  LOG(INFO, "[WiFi] Starting WiFiManager");
  wifi_.reset(
    new Esp32WiFiManager(ssid_.c_str(), password_.c_str()
                       , (openlcb::SimpleCanStack *)stack_
                       , cfg_.seg().wifi(), CONFIG_HOSTNAME_PREFIX, mode_
                       , stationIP_.get(), stationDNS_
                       , CONFIG_WIFI_SOFT_AP_CHANNEL));

  // When operating as both SoftAP and Station mode it is not necessary to wait
  // for the station to be UP during CS startup.
  if (mode_ == WIFI_MODE_APSTA)
  {
    wifi_->wait_for_ssid_connect(false);
  }

}

void LCCWiFiManager::shutdown()
{
  wifi_.reset(nullptr);
}

void LCCWiFiManager::reconfigure_mode(string mode, bool restart)
{
  auto cfg_mgr = Singleton<ConfigurationManager>::instance();
  if (!mode.compare(JSON_VALUE_WIFI_MODE_SOFTAP_ONLY))
  {
    cfg_mgr->remove(WIFI_STATION_CFG);
    string soft_ap_cfg = StringPrintf("%s\n%s", CONFIG_WIFI_SOFTAP_SSID
                                    , CONFIG_WIFI_SOFTAP_PASSWORD);
    cfg_mgr->store(WIFI_SOFTAP_CFG, soft_ap_cfg);
  }
  else if (!mode.compare(JSON_VALUE_WIFI_MODE_SOFTAP_STATION))
  {
    string soft_ap_cfg = StringPrintf("%s\n%s", CONFIG_WIFI_SOFTAP_SSID
                                    , CONFIG_WIFI_SOFTAP_PASSWORD);
    cfg_mgr->store(WIFI_SOFTAP_CFG, soft_ap_cfg);
  }
  else
  {
    cfg_mgr->remove(WIFI_SOFTAP_CFG);
  }
  if (restart)
  {
    stack_->executor()->add(new CallbackExecutable([]()
    {
      reboot();
    }));
  }
}

void LCCWiFiManager::reconfigure_station(string ssid, string password
                                       , string ip, string gateway
                                       , string subnet, string dns
                                       , bool restart)
{
  LOG(VERBOSE, "[WiFi] reconfigure_station(%s,%s,%s,%s,%s,%s,%d)", ssid.c_str()
    , password.c_str(), ip.c_str(), gateway.c_str(), subnet.c_str()
    , dns.c_str(), restart);
  auto cfg_mgr = Singleton<ConfigurationManager>::instance();
  string station_cfg = StringPrintf("%s\n%s", ssid.c_str(), password.c_str());
  cfg_mgr->store(WIFI_STATION_CFG, station_cfg);

  if (!ip.empty() && !gateway.empty() && !subnet.empty())
  {
    string ip_cfg = StringPrintf("%s\n%s\n%s\n", ip.c_str()
                                , gateway.c_str(), subnet.c_str());
    cfg_mgr->store(WIFI_STATION_IP_CFG, ip_cfg);
  }
  else
  {
    cfg_mgr->remove(WIFI_STATION_IP_CFG);
  }
  if (!dns.empty())
  {
    cfg_mgr->store(WIFI_STATION_DNS_CFG, dns);
  }
  else
  {
    cfg_mgr->remove(WIFI_STATION_DNS_CFG);
  }

  if (restart)
  {
    stack_->executor()->add(new CallbackExecutable([]()
    {
      reboot();
    }));
  }
}

string LCCWiFiManager::wifi_scan_json(bool ignore_duplicates)
{
  string result = "[";
  SyncNotifiable n;
  wifi_->start_ssid_scan(&n);
  n.wait_for_notification();
  size_t num_found = wifi_->get_ssid_scan_result_count();
  vector<string> seen_ssids;
  for (int i = 0; i < num_found; i++)
  {
    auto entry = wifi_->get_ssid_scan_result(i);
    if (ignore_duplicates)
    {
      if (std::find_if(seen_ssids.begin(), seen_ssids.end()
        , [entry](string &s)
          {
            return s == (char *)entry.ssid;
          }) != seen_ssids.end())
      {
        // filter duplicate SSIDs
        continue;
      }
      seen_ssids.push_back((char *)entry.ssid);
    }
    if (result.length() > 1)
    {
      result += ",";
    }
    LOG(VERBOSE, "auth:%d,rssi:%d,ssid:%s", entry.authmode, entry.rssi, entry.ssid);
    result += StringPrintf("{\"auth\":%d,\"rssi\":%d,\"ssid\":\"",
                           entry.authmode, entry.rssi);
    // TODO: remove this in favor of proper url encoding of non-ascii characters
    for (uint8_t idx = 0; idx < 33; idx++)
    {
      if (entry.ssid[idx] >= 0x20 && entry.ssid[idx] <= 0x7F)
      {
        result += entry.ssid[idx];
      }
      else if (entry.ssid[idx])
      {
        result += StringPrintf("%%%02x", entry.ssid[idx]);
      }
      else
      {
        // end of ssid
        break;
      }
    }
    result += "\"}";
  }
  result += "]";
  wifi_->clear_ssid_scan_results();
  return result;
}

string LCCWiFiManager::get_config_json()
{
  auto cfg_mgr = Singleton<ConfigurationManager>::instance();
  string config = StringPrintf("\"wifi\":{"
                                "\"mode\":\"%s\""
                              , mode_ == WIFI_MODE_AP ? "softap"
                              : mode_ == WIFI_MODE_APSTA ? "softap-station" : "station");
  if (mode_ != WIFI_MODE_AP)
  {
    config += StringPrintf(",\"station\":{\"ssid\":\"%s\",\"password\":\"%s\""
                         , ssid_.c_str(), password_.c_str());
    if (cfg_mgr->exists(WIFI_STATION_DNS_CFG))
    {
      config += ",\"dns\":\"" + cfg_mgr->load(WIFI_STATION_DNS_CFG) + "\"";
    }
    if (stationIP_.get() != nullptr)
    {
      config += StringPrintf(",\"ip\":\"" IPSTR "\",\"gateway\":\"" IPSTR "\","
                             "\"netmask\":\"" IPSTR "\""
                            , IP2STR(&stationIP_->ip), IP2STR(&stationIP_->gw)
                            , IP2STR(&stationIP_->netmask));
    }
    config += "}";
  }
  config += "}";
  return config;
}

} // namespace esp32cs