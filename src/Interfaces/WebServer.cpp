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

#include "interfaces/http/Dnsd.h"
#include "interfaces/http/Http.h"

// generated web content
#include "generated/index_html.h"
#include "generated/jquery_min_js.h"
#include "generated/jquery_mobile_js.h"
#include "generated/jquery_mobile_css.h"
#include "generated/jquery_simple_websocket.h"
#include "generated/jq_clock.h"
#include "generated/ajax_loader.h"
#include "generated/loco_32x32.h"

using esp32cs::http::Httpd;
using esp32cs::http::HttpMethod;
using esp32cs::http::HttpRequest;
using esp32cs::http::HttpStatusCode;
using esp32cs::http::AbstractHttpResponse;
using esp32cs::http::StringResponse;
using esp32cs::http::WebSocketFlow;
using esp32cs::http::MIME_TYPE_TEXT_HTML;
using esp32cs::http::MIME_TYPE_TEXT_JAVASCRIPT;
using esp32cs::http::MIME_TYPE_TEXT_PLAIN;
using esp32cs::http::MIME_TYPE_TEXT_XML;
using esp32cs::http::MIME_TYPE_TEXT_CSS;
using esp32cs::http::MIME_TYPE_IMAGE_PNG;
using esp32cs::http::MIME_TYPE_IMAGE_GIF;
using esp32cs::http::MIME_TYPE_APPLICATION_JSON;
using esp32cs::http::HTTP_ENCODING_GZIP;
using esp32cs::http::WebSocketEvent;

// Captive Portal landing page
static constexpr const char * const CAPTIVE_PORTAL_HTML = R"!^!(
<html>
 <head>
  <title>%s v%s</title>
  <meta http-equiv="refresh" content="30;url='/captiveauth'" />
 </head>
 <body>
  <h1>Welcome to the %s</h1>
  <h2>Open your browser and navigate to any website and it will open the %s</h2>
  <p>Click <a href="/captiveauth">here</a> to close this portal page if it does not automatically close.</p>
 </body>
</html>)!^!";

uninitialized<Httpd> httpd;
OSMutex webSocketLock;
vector<unique_ptr<WebSocketClient>> webSocketClients;
WEBSOCKET_STREAM_HANDLER(process_websocket_event);
HTTP_STREAM_HANDLER(process_ota);
HTTP_HANDLER(process_power);
HTTP_HANDLER(process_config);
HTTP_HANDLER(process_prog);
HTTP_HANDLER(process_turnouts);
HTTP_HANDLER(process_loco);
HTTP_HANDLER(process_outputs);
HTTP_HANDLER(process_sensors);
HTTP_HANDLER(process_remote_sensors);
HTTP_HANDLER(process_s88);

void init_webserver(MDNS *dns)
{
  httpd.emplace(dns);
  httpd->redirect_uri("/", "/index.html");
  // if the soft AP interface is enabled, setup the captive portal
  if (configStore->isAPEnabled())
  {
    httpd->captive_portal(
      StringPrintf(CAPTIVE_PORTAL_HTML, PROJECT_NAME, PROJECT_VER, PROJECT_NAME
                 , PROJECT_NAME));
  }
  httpd->static_uri("/index.html", indexHtmlGz, indexHtmlGz_size
                  , MIME_TYPE_TEXT_HTML, HTTP_ENCODING_GZIP);
  httpd->static_uri("/loco-32x32.png", loco32x32, loco32x32_size
                  , MIME_TYPE_IMAGE_PNG);
  httpd->static_uri("/jquery.min.js", jqueryJsGz, jqueryJsGz_size
                  , MIME_TYPE_TEXT_JAVASCRIPT, HTTP_ENCODING_GZIP);
  httpd->static_uri("/jquery.mobile-1.5.0-rc1.min.js", jqueryMobileJsGz
                  , jqueryMobileJsGz_size, MIME_TYPE_TEXT_JAVASCRIPT
                  , HTTP_ENCODING_GZIP);
  httpd->static_uri("/jquery.mobile-1.5.0-rc1.min.css", jqueryMobileCssGz
                  , jqueryMobileCssGz_size, MIME_TYPE_TEXT_CSS
                  , HTTP_ENCODING_GZIP);
  httpd->static_uri("/jquery.simple.websocket.min.js"
                  , jquerySimpleWebSocketGz, jquerySimpleWebSocketGz_size
                  , MIME_TYPE_TEXT_JAVASCRIPT, HTTP_ENCODING_GZIP);
  httpd->static_uri("/jqClock-lite.min.js", jqClockGz, jqClockGz_size
                  , MIME_TYPE_TEXT_JAVASCRIPT, HTTP_ENCODING_GZIP);
  httpd->static_uri("/images/ajax-loader.gif", ajaxLoader, ajaxLoader_size
                  , MIME_TYPE_IMAGE_GIF);
  httpd->websocket_uri("/ws", process_websocket_event);
  httpd->uri("/update", HttpMethod::POST, nullptr, process_ota);
  httpd->uri("/features", [&](HttpRequest *req)
  {
    return new StringResponse(configStore->getCSFeatures()
                            , MIME_TYPE_APPLICATION_JSON);
  });
  httpd->uri("/fs", HttpMethod::GET, [&](HttpRequest *req)
  {
    string path = req->param("path");
    string data = read_file_to_string(path);
    string mimetype;
    if (path.find(".xml") != string::npos)
    {
      mimetype = MIME_TYPE_TEXT_XML;
    }
    else if (path.find(".json") != string::npos)
    {
      mimetype = MIME_TYPE_APPLICATION_JSON;
    }
    return new StringResponse(data, mimetype);
  });
  httpd->uri("/power", HttpMethod::GET | HttpMethod::PUT, process_power);
  httpd->uri("/config", HttpMethod::GET | HttpMethod::POST, process_config);
  httpd->uri("/programmer", HttpMethod::GET | HttpMethod::POST, process_prog);
  httpd->uri("/turnouts"
           , HttpMethod::GET | HttpMethod::POST |
             HttpMethod::PUT | HttpMethod::DELETE
           , process_turnouts);
  httpd->uri("/locomotive"
           , HttpMethod::GET | HttpMethod::POST |
             HttpMethod::PUT | HttpMethod::DELETE
           , process_loco);
  httpd->uri("/locomotive/roster"
           , HttpMethod::GET | HttpMethod::POST |
             HttpMethod::PUT | HttpMethod::DELETE
           , process_loco);
  httpd->uri("/locomotive/estop"
           , HttpMethod::GET | HttpMethod::POST |
             HttpMethod::PUT | HttpMethod::DELETE
           , process_loco);
#if ENABLE_OUTPUTS
  httpd->uri("/outputs"
           , HttpMethod::GET | HttpMethod::POST |
             HttpMethod::PUT | HttpMethod::DELETE
           , process_outputs);
#endif // ENABLE_OUTPUTS
#if ENABLE_SENSORS
  httpd->uri("/sensors"
          , HttpMethod::GET | HttpMethod::POST | HttpMethod::DELETE
          , process_sensors);
  httpd->uri("/remoteSensors"
          , HttpMethod::GET | HttpMethod::POST | HttpMethod::DELETE
          , process_remote_sensors);
#if S88_ENABLED
  httpd->uri("/s88sensors"
          , HttpMethod::GET | HttpMethod::POST | HttpMethod::DELETE
          , process_s88);
#endif // S88_ENABLED
#endif // ENABLE_SENSORS
}

void to_json(nlohmann::json& j, const wifi_ap_record_t& t)
{
  j = nlohmann::json({
    { JSON_WIFI_SSID_NODE, (char *)t.ssid },
    { JSON_WIFI_RSSI_NODE, t.rssi },
    { JSON_WIFI_AUTH_NODE, t.authmode },
  });
}

WEBSOCKET_STREAM_HANDLER_IMPL(process_websocket_event, client, event, data
                            , data_len)
{
  OSMutexLock h(&webSocketLock);
  if (event == WebSocketEvent::WS_EVENT_CONNECT)
  {
    webSocketClients.push_back(
      esp32cs::make_unique<WebSocketClient>(client->id(), client->ip()));
    Singleton<InfoScreen>::instance()->replaceLine(
      INFO_SCREEN_CLIENTS_LINE, "TCP Conn: %02d"
    , webSocketClients.size() + jmriClients.size());
  }
  else if (event == WebSocketEvent::WS_EVENT_DISCONNECT)
  {
    webSocketClients.erase(std::remove_if(webSocketClients.begin()
                                        , webSocketClients.end()
    , [client](unique_ptr<WebSocketClient> &inst) -> bool
      {
        return inst->id() == client->id();
      }));
    Singleton<InfoScreen>::instance()->replaceLine(
      INFO_SCREEN_CLIENTS_LINE, "TCP Conn: %02d",
      webSocketClients.size() + jmriClients.size());
  }
  else if (event == WebSocketEvent::WS_EVENT_TEXT)
  {
    auto ent = std::find_if(webSocketClients.begin(), webSocketClients.end()
    , [client](unique_ptr<WebSocketClient> &inst) -> bool
      {
        return inst->id() == client->id();
      }
    );
    if (ent != webSocketClients.end())
    {
      // TODO: remove cast that drops const
      auto res = (*ent)->feed((uint8_t *)data, data_len);
      if (res.length())
      {
        client->send_text(res);
      }
    }
  }
}

esp_ota_handle_t otaHandle;
esp_partition_t *ota_partition = nullptr;
HTTP_STREAM_HANDLER_IMPL(process_ota, request, filename, size, data, length
                       , offset, final, abort_req)
{
  esp_err_t err = ESP_OK;
  if (!offset)
  {
    esp_log_level_set("esp_image", ESP_LOG_VERBOSE);
    ota_partition = (esp_partition_t *)esp_ota_get_next_update_partition(NULL);
    err = ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ota_begin(ota_partition, size
                                                    , &otaHandle));
    if (err != ESP_OK)
    {
      LOG_ERROR("[WebSrv] OTA start failed, aborting!");
      Singleton<OTAMonitorFlow>::instance()->report_failure(err);
      request->set_status(HttpStatusCode::STATUS_SERVER_ERROR);
      *abort_req = true;
      return nullptr;
    }
    LOG(INFO, "[WebSrv] OTA Update starting (%zu bytes, target:%s)...", size
      , ota_partition->label);
    trackSignal->disable_ops_output();
    trackSignal->disable_prog_output();
    Singleton<OTAMonitorFlow>::instance()->report_start();
  }
  HASSERT(ota_partition);
  ESP_ERROR_CHECK(esp_ota_write(otaHandle, data, length));
  Singleton<OTAMonitorFlow>::instance()->report_progress(length);
  if (final)
  {
    err = ESP_ERROR_CHECK_WITHOUT_ABORT(esp_ota_end(otaHandle));
    if (err != ESP_OK)
    {
      LOG_ERROR("[WebSrv] OTA end failed, aborting!");
      Singleton<OTAMonitorFlow>::instance()->report_failure(err);
      request->set_status(HttpStatusCode::STATUS_SERVER_ERROR);
      *abort_req = true;
      return nullptr;
    }
    LOG(INFO, "[WebSrv] OTA binary received, setting boot partition: %s"
      , ota_partition->label);
    err = ESP_ERROR_CHECK_WITHOUT_ABORT(
      esp_ota_set_boot_partition(ota_partition));
    if (err != ESP_OK)
    {
      LOG_ERROR("[WebSrv] OTA end failed, aborting!");
      Singleton<OTAMonitorFlow>::instance()->report_failure(err);
      request->set_status(HttpStatusCode::STATUS_SERVER_ERROR);
      *abort_req = true;
      return nullptr;
    }
    LOG(INFO, "[WebSrv] OTA Update Complete!");
    Singleton<OTAMonitorFlow>::instance()->report_success();
    request->set_status(HttpStatusCode::STATUS_OK);
    return new StringResponse("OTA Upload Complete", MIME_TYPE_TEXT_PLAIN);
  }
  return nullptr;
}

HTTP_HANDLER_IMPL(process_power, request)
{
  string response = "{}";
  request->set_status(HttpStatusCode::STATUS_OK);
  if (request->method() == HttpMethod::GET)
  {
    response.assign(trackSignal->generate_status_json());
  }
  else if (request->method() == HttpMethod::PUT)
  {
    if (request->param(JSON_STATE_NODE, false))
    {
      trackSignal->enable_ops_output();
    }
    else
    {
      trackSignal->disable_ops_output();
      trackSignal->disable_prog_output();
    }
  }
  return new StringResponse(response, MIME_TYPE_APPLICATION_JSON);
}

HTTP_HANDLER_IMPL(process_config, request)
{
  bool needReboot = false;
  if (request->has_param("reset"))
  {
    configStore->factory_reset();
    needReboot = true;
  }
  else if (request->has_param("scan"))
  {
    SyncNotifiable n;
    Esp32WiFiManager *wifi_mgr = Singleton<Esp32WiFiManager>::instance();
    wifi_mgr->start_ssid_scan(&n);
    n.wait_for_notification();
    size_t num_found = wifi_mgr->get_ssid_scan_result_count();
    nlohmann::json ssid_list;
    vector<string> seen_ssids;
    for (int i = 0; i < num_found; i++)
    {
      auto entry = wifi_mgr->get_ssid_scan_result(i);
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
      ssid_list.push_back(entry);
    }
    wifi_mgr->clear_ssid_scan_results();
    return new StringResponse(ssid_list.dump(), MIME_TYPE_APPLICATION_JSON);
  }
  if (request->has_param("ssid") &&
      configStore->setWiFiStationParams(request->param("ssid")
                                      , request->param("password")))
  {
    needReboot = true;
  }
  if (request->has_param("mode") &&
      configStore->setWiFiMode(request->param("mode")))
  {
    needReboot = true;
  }
  if (request->has_param("nodeid") &&
      configStore->setNodeID(request->param("nodeid")))
  {
    needReboot = true;
  }
  if (request->has_param("lcc-can") &&
      configStore->setLCCCan(request->param("lcc-can", false)))
  {
    needReboot = true;
  }
  if (request->has_param("lcc-hub"))
  {
    configStore->setLCCHub(request->param("lcc-hub", false));
  }
  if (request->has_param("uplink-mode") && request->has_param("uplink-service") &&
      request->has_param("uplink-manual") && request->has_param("uplink-manual-port"))
  {
    // WiFi uplink settings do not require a reboot
    configStore->setWiFiUplinkParams((SocketClientParams::SearchMode)request->param("uplink-mode"
                                    , SocketClientParams::SearchMode::AUTO_MANUAL)
                                    , request->param("uplink-service")
                                    , request->param("uplink-manual")
                                    , request->param("uplink-manual-port"
                                                   , openlcb::TcpClientDefaultParams::DEFAULT_PORT));
  }
  if (request->has_param("ops-short") && request->has_param("ops-short-clear") &&
      request->has_param("ops-shutdown") && request->has_param("ops-shutdown-clear") &&
      request->has_param("ops-thermal") && request->has_param("ops-thermal-clear"))
  {
    configStore->setHBridgeEvents(OPS_CDI_TRACK_OUTPUT_INDEX
                                , request->param("ops-short")
                                , request->param("ops-short-clear")
                                , request->param("ops-shutdown")
                                , request->param("ops-shutdown-clear")
                                , request->param("ops-thermal")
                                , request->param("ops-thermal-clear"));
  }
  if (request->has_param("prog-short") && request->has_param("prog-short-clear") &&
      request->has_param("prog-shutdown") && request->has_param("prog-shutdown-clear"))
  {
    configStore->setHBridgeEvents(PROG_CDI_TRACK_OUTPUT_INDEX
                                , request->param("prog-short")
                                , request->param("prog-short-clear")
                                , request->param("prog-shutdown")
                                , request->param("prog-shutdown-clear"));
  }

  if (needReboot)
  {
    // send a string back to the client rather than SEND_GENERIC_RESPONSE
    // so we don't return prior to calling reboot.
    return new StringResponse("{restart:\"ESP32CommandStation Restarting!\"}"
                            , MIME_TYPE_APPLICATION_JSON);
  }

  return new StringResponse(configStore->getCSConfig()
                          , MIME_TYPE_APPLICATION_JSON);
}

HTTP_HANDLER_IMPL(process_prog, request)
{
  request->set_status(HttpStatusCode::STATUS_OK);
  if (!request->has_param(JSON_PROG_ON_MAIN))
  {
    request->set_status(HttpStatusCode::STATUS_BAD_REQUEST);
  }
  else if (request->method() == HttpMethod::GET)
  {
    if (request->param(JSON_PROG_ON_MAIN, false))
    {
      request->set_status(HttpStatusCode::STATUS_NOT_ALLOWED);
    }
    else if(request->has_param(JSON_IDENTIFY_NODE))
    {
      int16_t decoderConfig = readCV(CV_NAMES::DECODER_CONFIG);
      if (decoderConfig > 0)
      {
        uint16_t decoderAddress = 0;
        string response = "{";
        if (bitRead(decoderConfig, DECODER_CONFIG_BITS::DECODER_TYPE))
        {
          uint8_t decoderManufacturer = readCV(CV_NAMES::DECODER_MANUFACTURER);
          int16_t addrMSB = readCV(CV_NAMES::ACCESSORY_DECODER_MSB_ADDRESS);
          int16_t addrLSB = readCV(CV_NAMES::SHORT_ADDRESS);
          if (addrMSB >= 0 && addrLSB >= 0)
          {
            if (decoderManufacturer == 0xA5)
            {
              // MERG uses 7 bit LSB
              decoderAddress = (uint16_t)(((addrMSB & 0x07) << 7) | (addrLSB & 0x7F));
            }
            else if(decoderManufacturer == 0x19)
            {
              // Team Digital uses 8 bit LSB and 4 bit MSB
              decoderAddress = (uint16_t)(((addrMSB & 0x0F) << 8) | addrLSB);
            }
            else
            {
              // NMRA spec shows 6 bit LSB
              decoderAddress = (uint16_t)(((addrMSB & 0x07) << 6) | (addrLSB & 0x1F));
            }
            response += StringPrintf("\"%s\":\"%s\",", JSON_ADDRESS_MODE_NODE
                                   , JSON_VALUE_LONG_ADDRESS);
          }
          else
          {
            LOG(WARNING, "Failed to read address MSB/LSB");
            request->set_status(HttpStatusCode::STATUS_SERVER_ERROR);
          }
        }
        else
        {
          if (bitRead(decoderConfig, DECODER_CONFIG_BITS::SHORT_OR_LONG_ADDRESS))
          {
            int16_t addrMSB = readCV(CV_NAMES::LONG_ADDRESS_MSB_ADDRESS);
            int16_t addrLSB = readCV(CV_NAMES::LONG_ADDRESS_LSB_ADDRESS);
            if (addrMSB >= 0 && addrLSB >= 0)
            {
              decoderAddress = (uint16_t)(((addrMSB & 0xFF) << 8) | (addrLSB & 0xFF));
              response += StringPrintf("\"%s\":\"%s\",", JSON_ADDRESS_MODE_NODE
                                     , JSON_VALUE_LONG_ADDRESS);

            }
            else
            {
              LOG(WARNING, "Unable to read address MSB/LSB");
              request->set_status(HttpStatusCode::STATUS_SERVER_ERROR);
            }
          }
          else
          {
            int16_t shortAddr = readCV(CV_NAMES::SHORT_ADDRESS);
            if (shortAddr > 0)
            {
              decoderAddress = shortAddr;
              response += StringPrintf("\"%s\":\"%s\",", JSON_ADDRESS_MODE_NODE
                                     , JSON_VALUE_SHORT_ADDRESS);
            }
            else
            {
              LOG(WARNING, "Unable to read short address CV");
              request->set_status(HttpStatusCode::STATUS_SERVER_ERROR);
            }
          }
          response += StringPrintf("\"%s\":%s,", JSON_SPEED_TABLE_NODE
                                 , bitRead(decoderConfig
                                         , DECODER_CONFIG_BITS::SPEED_TABLE) ?
                                           JSON_VALUE_ON : JSON_VALUE_OFF);
        }
        response += StringPrintf("\"%s\":%d,", JSON_ADDRESS_NODE
                               , decoderAddress);
        // if it is a mobile decoder *AND* we are requested to create it, send
        // the decoder address to the train db to create an entry.
        if (request->param(JSON_CREATE_NODE, false) &&
           (decoderConfig & BIT(DECODER_CONFIG_BITS::DECODER_TYPE)) == 0)
        {
          auto traindb = Singleton<esp32cs::Esp32TrainDatabase>::instance();
          traindb->create_if_not_found(decoderAddress);
        }
        response += "}";
        return new StringResponse(response, MIME_TYPE_APPLICATION_JSON);
      }
      else
      {
        LOG(WARNING, "Failed to read decoder configuration");
        request->set_status(HttpStatusCode::STATUS_SERVER_ERROR);
      }
    }
    else
    {
      uint16_t cvNumber = request->param(JSON_CV_NODE, 0);
      if (cvNumber == 0)
      {
        request->set_status(HttpStatusCode::STATUS_BAD_REQUEST);
      }
      else
      {
        int16_t cvValue = readCV(cvNumber);
        if (cvValue < 0)
        {
          request->set_status(HttpStatusCode::STATUS_SERVER_ERROR);
        }
        else
        {
          return new StringResponse(
            StringPrintf("{\"%s\":%d,\"%s\":%d}"
                      , JSON_CV_NODE, cvNumber
                      , JSON_VALUE_NODE, cvValue)
          , MIME_TYPE_APPLICATION_JSON);
        }
      }
    }
  }
  else if (request->method() == HttpMethod::POST)
  {
    uint16_t cv_num = request->param(JSON_CV_NODE, 0);
    uint16_t cv_value = 0;
    uint8_t cv_bit = request->param(JSON_CV_BIT_NODE, 0);
    bool pom = request->param(JSON_PROG_ON_MAIN, false);
    if (request->has_param(JSON_CV_BIT_NODE))
    {
      cv_value = request->param(JSON_VALUE_NODE, false);
    }
    else
    {
      cv_value = request->param(JSON_VALUE_NODE, 0);
    }

    if (cv_num == 0)
    {
      request->set_status(HttpStatusCode::STATUS_BAD_REQUEST);
    }
    else if (pom)
    {
      uint16_t address = request->param(JSON_ADDRESS_NODE, 0);
      if (request->has_param(JSON_CV_BIT_NODE))
      {
        writeOpsCVBit(address, cv_num, cv_bit, cv_value);
      }
      else
      {
        writeOpsCVByte(address, cv_num, cv_value);
      }
    }
    else if (request->has_param(JSON_CV_BIT_NODE) &&
             !writeProgCVBit(cv_num, cv_bit,cv_value))
    {
      request->set_status(HttpStatusCode::STATUS_SERVER_ERROR);
    }
    else if (!writeProgCVByte(cv_num, cv_value))
    {
      request->set_status(HttpStatusCode::STATUS_SERVER_ERROR);
    }
  }
  return nullptr;
}

// GET /turnouts - full list of turnouts, note that turnout state is STRING type for display
// GET /turnouts?readbleStrings=[0,1] - full list of turnouts, turnout state will be returned as true/false (boolean) when readableStrings=0.
// GET /turnouts?address=<address> - retrieve turnout by DCC address
// GET /turnouts?id=<id> - retrieve turnout by ID
// PUT /turnouts?address=<address> - toggle turnout by DCC address
// PUT /turnouts?id=<id> - toggle turnout by ID
// POST /turnouts?id=<id>&address=<address>&subAddress=<subAddress>&type=<type> - creates a new turnout
//      when id is -1 it will be auto-assigned as number of defined turnouts + 1.
//      if subAddress is -1 the turnout will be defined as DCC address only.
// DELETE /turnouts?id=<id> - delete turnout by ID
// DELETE /turnouts?address=<address> - delete turnout by DCC address
//
// For successful requests the result code will be 200 and either an array of turnouts or single turnout will be returned.
// For unsuccessful requests the result code will be 400 (bad request, missing args), 401 (not found), 500 (server failure).
//
HTTP_HANDLER_IMPL(process_turnouts, request)
{
  request->set_status(HttpStatusCode::STATUS_SERVER_ERROR);
  if (request->method() == HttpMethod::GET &&
     !request->has_param(JSON_ID_NODE) &&
     !request->has_param(JSON_ADDRESS_NODE))
  {
    bool readable = true;
    if (request->has_param(JSON_TURNOUTS_READABLE_STRINGS_NODE))
    {
      readable = request->param(JSON_TURNOUTS_READABLE_STRINGS_NODE, 0);
    }
    return new StringResponse(turnoutManager->getStateAsJson(readable)
                            , MIME_TYPE_APPLICATION_JSON);
  }

  int16_t id = request->param(JSON_ID_NODE, -1);
  uint16_t address = request->param(JSON_ADDRESS_NODE, 0);
  uint16_t subaddress = request->param(JSON_SUB_ADDRESS_NODE, 0);
  if (request->method() == HttpMethod::GET)
  {
    if (request->has_param(JSON_ID_NODE))
    {
      auto turnout = turnoutManager->getTurnoutByID(id);
      if (turnout)
      {
        return new StringResponse(turnout->toJson()
                                , MIME_TYPE_APPLICATION_JSON);
      }
      request->set_status(HttpStatusCode::STATUS_NOT_FOUND);
    }
    else
    {
      auto turnout = turnoutManager->getTurnoutByID(address);
      if (turnout)
      {
        return new StringResponse(turnout->toJson()
                                , MIME_TYPE_APPLICATION_JSON);
      }
      request->set_status(HttpStatusCode::STATUS_NOT_FOUND);
    }
  }
  else if (request->method() == HttpMethod::POST)
  {
    TurnoutType type = (TurnoutType)request->param(JSON_TYPE_NODE
                                                 , TurnoutType::LEFT);
    // auto generate ID
    if (id == -1)
    {
      id = turnoutManager->getTurnoutCount() + 1;
    }
    auto turnout = turnoutManager->createOrUpdate((uint16_t)id, address
                                                , subaddress, type);
    if (turnout)
    {
      return new StringResponse(turnout->toJson(), MIME_TYPE_APPLICATION_JSON);
    }
    request->set_status(HttpStatusCode::STATUS_NOT_FOUND);
  }
  else if (request->method() == HttpMethod::DELETE)
  {
    if (request->has_param(JSON_ID_NODE) && turnoutManager->removeByID(id))
    {
      request->set_status(HttpStatusCode::STATUS_OK);
    }
    else if (turnoutManager->removeByAddress(address))
    {
      request->set_status(HttpStatusCode::STATUS_OK);
    }
  }
  else if (request->method() == HttpMethod::PUT)
  {
    if (request->has_param(JSON_ID_NODE))
    {
      turnoutManager->toggleByID(id);
    }
    else
    {
      turnoutManager->toggleByAddress(address);
    }
    request->set_status(HttpStatusCode::STATUS_OK);
  }
  return nullptr;
}

string convert_loco_to_json(TrainImpl *t)
{
  if (!t)
  {
    return "{}";
  }
  nlohmann::json j =
  {
    { JSON_ADDRESS_NODE, t->legacy_address() },
    { JSON_SPEED_NODE, t->get_speed().get_dcc_128() & 0x7F },
    { JSON_DIRECTION_NODE, t->get_speed().direction() ? JSON_VALUE_REVERSE : JSON_VALUE_FORWARD},
  };
  for(uint8_t funcID = 0; funcID < DCC_MAX_FN; funcID++)
  {
    j[JSON_FUNCTIONS_NODE].push_back({
      { JSON_ID_NODE, funcID },
      { JSON_STATE_NODE, t->get_fn(funcID) }
    });
  }
  return j.dump();
}

#define GET_LOCO_VIA_EXECUTOR(NAME, address)                                          \
  TrainImpl *NAME = nullptr;                                                          \
  {                                                                                   \
    SyncNotifiable n;                                                                 \
    extern unique_ptr<OpenMRN> openmrn;                                               \
    openmrn->stack()->executor()->add(new CallbackExecutable(                         \
    [&]()                                                                             \
    {                                                                                 \
      NAME = trainNodes->get_train_impl(commandstation::DccMode::DCC_128_LONG_ADDRESS \
                                      , address);                                     \
      n.notify();                                                                     \
    }));                                                                              \
    n.wait_for_notification();                                                        \
  }

// method - url pattern - meaning
// ANY /locomotive/estop - send emergency stop to all locomotives
// GET /locomotive/roster - roster
// GET /locomotive/roster?address=<address> - get roster entry
// PUT /locomotive/roster?address=<address> - update roster entry
// POST /locomotive/roster?address=<address> - create roster entry
// DELETE /locomotive/roster?address=<address> - delete roster entry
// GET /locomotive - get active locomotives
// POST /locomotive?address=<address> - add locomotive to active management
// GET /locomotive?address=<address> - get locomotive state
// PUT /locomotive?address=<address>&speed=<speed>&dir=[FWD|REV]&fX=[true|false] - Update locomotive state, fX is short for function X where X is 0-28.
// DELETE /locomotive?address=<address> - removes locomotive from active management
HTTP_HANDLER_IMPL(process_loco, request)
{
  string url = request->uri();
  auto traindb = Singleton<esp32cs::Esp32TrainDatabase>::instance();
  request->set_status(HttpStatusCode::STATUS_BAD_REQUEST);

  // check if we have an eStop command, we don't care how this gets sent to the
  // command station (method) so check it first
  if(url.find("/estop") != string::npos)
  {
    Singleton<esp32cs::EStopHandler>::instance()->set_state(true);
    request->set_status(HttpStatusCode::STATUS_OK);
  }
  else if(url.find("/roster")  != string::npos)
  {
    if(request->method() == HttpMethod::GET &&
       !request->has_param(JSON_ADDRESS_NODE))
    {
      return new StringResponse(
        Singleton<esp32cs::Esp32TrainDatabase>::instance()->get_all_entries_as_json()
      , MIME_TYPE_APPLICATION_JSON);
    }
    else if (request->has_param(JSON_ADDRESS_NODE))
    {
      if(request->method() == HttpMethod::DELETE)
      {
        traindb->delete_entry(request->param(JSON_ADDRESS_NODE, 0));
        request->set_status(HttpStatusCode::STATUS_OK);
      }
      else
      {
        uint16_t address = request->param(JSON_ADDRESS_NODE, 0);
        traindb->create_if_not_found(address);
        if(request->method() == HttpMethod::PUT ||
           request->method() == HttpMethod::POST)
        {
          if(request->has_param(JSON_NAME_NODE))
          {
            string name = request->param(JSON_NAME_NODE);
            traindb->set_train_name(address, name);
          }
          if(request->has_param(JSON_IDLE_ON_STARTUP_NODE))
          {
            traindb->set_train_auto_idle(address
                                       , request->param(JSON_IDLE_ON_STARTUP_NODE
                                                      , false));
          }
          if(request->has_param(JSON_DEFAULT_ON_THROTTLE_NODE))
          {
            bool value = request->param(JSON_DEFAULT_ON_THROTTLE_NODE, false);
            traindb->set_train_show_on_limited_throttle(address, value);
          }
        }
        return new StringResponse(traindb->get_entry_as_json(address)
                                , MIME_TYPE_APPLICATION_JSON);
      }
    }
  }
  else
  {
    // Since it is not an eStop or roster command we need to check the request
    // method and ensure it contains the required arguments otherwise the
    // request should be rejected
    if(request->method() == HttpMethod::GET && 
       !request->has_param(JSON_ADDRESS_NODE))
    {
      // get all active locomotives
      string res = "[";
      for (size_t id = 0; id < trainNodes->size(); id++)
      {
        auto nodeid = trainNodes->get_train_node_id(id);
        if (nodeid)
        {
          if (res.length() > 1)
          {
            res += ",";
          }
          auto loco = trainNodes->get_train_impl(nodeid);
          res += convert_loco_to_json(loco);
        }
      }
      res += "]";
      return new StringResponse(res, MIME_TYPE_APPLICATION_JSON);
    }
    else if (request->has_param(JSON_ADDRESS_NODE))
    {
      uint16_t address = request->param(JSON_ADDRESS_NODE, 0);
      if(request->method() == HttpMethod::PUT ||
         request->method() == HttpMethod::POST)
      {
        GET_LOCO_VIA_EXECUTOR(loco, address);
        auto upd_speed = loco->get_speed();
        // Creation / Update of active locomotive
        if(request->has_param(JSON_IDLE_NODE))
        {
          upd_speed.set_dcc_128(0);
        }
        if(request->has_param(JSON_SPEED_NODE))
        {
          upd_speed.set_dcc_128(request->param(JSON_SPEED_NODE, 0));
        }
        if(request->has_param(JSON_DIRECTION_NODE))
        {
          bool forward =
            !request->param(JSON_DIRECTION_NODE).compare(JSON_VALUE_FORWARD);
          upd_speed.set_direction(forward ? dcc::SpeedType::FORWARD
                                          : dcc::SpeedType::REVERSE);
        }
        loco->set_speed(upd_speed);
        for(uint8_t funcID = 0; funcID <=28 ; funcID++)
        {
          string fArg = StringPrintf("f%d", funcID);
          if(request->has_param(fArg.c_str()))
          {
            loco->set_fn(funcID, request->param(fArg, false));
          }
        }
        return new StringResponse(convert_loco_to_json(loco)
                                , MIME_TYPE_APPLICATION_JSON);
      }
      else if(request->method() == HttpMethod::DELETE)
      {
        // we don't need to queue it up on the executor as it is done internally.
        trainNodes->remove_train_impl(address);
#if NEXTION_ENABLED
        static_cast<NextionThrottlePage *>(nextionPages[THROTTLE_PAGE])->invalidateLocomotive(address);
#endif
        request->set_status(HttpStatusCode::STATUS_NO_CONTENT);
      }
      else
      {
        GET_LOCO_VIA_EXECUTOR(loco, address);
        return new StringResponse(convert_loco_to_json(loco)
                                , MIME_TYPE_APPLICATION_JSON);
      }
    }
  }
  return nullptr;
}

#if ENABLE_OUTPUTS
HTTP_HANDLER_IMPL(process_outputs, request)
{
  if (request->method() == HttpMethod::GET && !request->params())
  {
    return new StringResponse(OutputManager::getStateAsJson()
                            , MIME_TYPE_APPLICATION_JSON);
  }
  request->set_status(HttpStatusCode::STATUS_OK);
  int16_t output_id = request->param(JSON_ID_NODE, -1);
  if (output_id < 0)
  {
    request->set_status(HttpStatusCode::STATUS_BAD_REQUEST);
  }
  else if (request->method() == HttpMethod::GET)
  {
    auto output = OutputManager::getOutput(output_id);
    if (output)
    {
      return new StringResponse(output->toJson(), MIME_TYPE_APPLICATION_JSON);
    }
    request->set_status(HttpStatusCode::STATUS_NOT_FOUND);
  }
  else if (request->method() == HttpMethod::POST)
  {
    int8_t pin = request->param(JSON_PIN_NODE, -1);
    bool inverted = request->param(JSON_INVERTED_NODE, false);
    bool forceState = request->param(JSON_FORCE_STATE_NODE, false);
    bool defaultState = request->param(JSON_DEFAULT_STATE_NODE, false);
    uint8_t outputFlags = 0;
    if (inverted)
    {
      bitSet(outputFlags, OUTPUT_IFLAG_INVERT);
    }
    if (forceState)
    {
      bitSet(outputFlags, OUTPUT_IFLAG_RESTORE_STATE);
      if (defaultState)
      {
        bitSet(outputFlags, OUTPUT_IFLAG_FORCE_STATE);
      }
    }
    if (pin < 0)
    {
      request->set_status(HttpStatusCode::STATUS_BAD_REQUEST);
    }
    else if (!OutputManager::createOrUpdate(output_id, pin, outputFlags))
    {
      request->set_status(HttpStatusCode::STATUS_NOT_ALLOWED);
    }
  }
  else if (request->method() == HttpMethod::DELETE &&
          !OutputManager::remove(output_id))
  {
    request->set_status(HttpStatusCode::STATUS_NOT_FOUND);
  }
  else if(request->method() == HttpMethod::PUT)
  {
    OutputManager::toggle(output_id);
  }
  return nullptr;
}
#endif // ENABLE_OUTPUTS

#if ENABLE_SENSORS
HTTP_HANDLER_IMPL(process_sensors, request)
{
  request->set_status(HttpStatusCode::STATUS_SERVER_ERROR);
  if (request->method() == HttpMethod::GET &&
      !request->has_param(JSON_ID_NODE))
  {
    return new StringResponse(SensorManager::getStateAsJson()
                            , MIME_TYPE_APPLICATION_JSON);
  }
  else if (!request->has_param(JSON_ID_NODE))
  {
    request->set_status(HttpStatusCode::STATUS_BAD_REQUEST);
  }
  else
  {
    int16_t id = request->param(JSON_ID_NODE, -1);
    if (id < 0)
    {
      request->set_status(HttpStatusCode::STATUS_BAD_REQUEST);
    }
    else if (request->method() == HttpMethod::GET)
    {
      auto sensor = SensorManager::getSensor(id);
      if (sensor)
      {
        return new StringResponse(sensor->toJson()
                                , MIME_TYPE_APPLICATION_JSON);
      }
      request->set_status(HttpStatusCode::STATUS_NOT_FOUND);
    }
    else if (request->method() == HttpMethod::POST)
    {
      int8_t pin = request->param(JSON_PIN_NODE, -1);
      bool pull = request->param(JSON_PULLUP_NODE, false);
      if (pin < 0)
      {
        request->set_status(HttpStatusCode::STATUS_BAD_REQUEST);
      }
      else if (!SensorManager::createOrUpdate(id, pin, pull))
      {
        request->set_status(HttpStatusCode::STATUS_NOT_ALLOWED);
      }
      else
      {
        request->set_status(HttpStatusCode::STATUS_OK);
      }
    }
    else if (request->method() == HttpMethod::DELETE)
    {
      if (SensorManager::getSensorPin(id) < 0)
      {
        // attempt to delete S88/RemoteSensor
        request->set_status(HttpStatusCode::STATUS_NOT_ALLOWED);
      }
      else if (!SensorManager::remove(id))
      {
        request->set_status(HttpStatusCode::STATUS_NOT_FOUND);
      }
      else
      {
        request->set_status(HttpStatusCode::STATUS_OK);
      }
    }
  }
  
  return nullptr;
}

HTTP_HANDLER_IMPL(process_remote_sensors, request)
{
  request->set_status(HttpStatusCode::STATUS_OK);
  if(request->method() == HttpMethod::GET)
  {
    return new StringResponse(RemoteSensorManager::getStateAsJson()
                            , MIME_TYPE_APPLICATION_JSON);
  }
  else if(request->method() == HttpMethod::POST)
  {
    RemoteSensorManager::createOrUpdate(request->param(JSON_ID_NODE, 0),
                                        request->param(JSON_VALUE_NODE, 0));
  }
  else if(request->method() == HttpMethod::DELETE)
  {
    RemoteSensorManager::remove(request->param(JSON_ID_NODE, 0));
  }
  return nullptr;
}

#if S88_ENABLED
HTTP_HANDLER_IMPL(process_s88, request)
{
  request->set_status(HttpStatusCode::STATUS_OK);
  if(request->method() == HttpMethod::GET)
  {
    return new StringResponse(S88BusManager::getStateAsJson()
                            , MIME_TYPE_APPLICATION_JSON);
  }
  else if(request->method() == HttpMethod::POST)
  {
    if(!S88BusManager::createOrUpdateBus(
      request->param(JSON_ID_NODE, 0),
      request->param(JSON_PIN_NODE, 0),
      request->param(JSON_COUNT_NODE, 0)))
    {
      // duplicate pin/id
      request->set_status(HttpStatusCode::STATUS_NOT_ALLOWED);
    }
  }
  else if(request->method() == HttpMethod::DELETE)
  {
    S88BusManager::removeBus(request->param(JSON_ID_NODE, 0));
  }
  return nullptr;
}
#endif // S88_ENABLED

#endif // ENABLE_SENSORS
