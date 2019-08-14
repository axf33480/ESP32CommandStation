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

#include <ESPAsyncDNSServer.h>

// generated web content
#include "generated/index_html.h"
#include "generated/jquery_min_js.h"
#include "generated/jquery_mobile_js.h"
#include "generated/jquery_mobile_css.h"
#include "generated/jquery_simple_websocket.h"
#include "generated/jq_clock.h"
#include "generated/ajax_loader.h"

AsyncWebServer webServer(80);
AsyncDNSServer asyncDNS;
AsyncWebSocket webSocket("/ws");

static constexpr const char * const APPLICATION_JSON_TYPE = "application/json";

enum HTTP_STATUS_CODES
{
  STATUS_OK = 200
, STATUS_NO_CONTENT = 204
, STATUS_NOT_MODIFIED = 304
, STATUS_BAD_REQUEST = 400
, STATUS_NOT_FOUND = 404
, STATUS_NOT_ALLOWED = 405
, STATUS_NOT_ACCEPTABLE = 406
, STATUS_CONFLICT = 409
, STATUS_PRECONDITION_FAILED = 412
, STATUS_SERVER_ERROR = 500
};

Atomic webSocketAtomic;
vector<unique_ptr<WebSocketClient>> webSocketClients;

void handleWsEvent(AsyncWebSocket * server,
                   AsyncWebSocketClient * client,
                   AwsEventType type,
                   void * arg,
                   uint8_t *data,
                   size_t len)
{
  AtomicHolder h(&webSocketAtomic);
  if (type == WS_EVT_CONNECT)
  {
    webSocketClients.emplace_back(
      new WebSocketClient(client->id(), ntohl(client->remoteIP())));
    Singleton<InfoScreen>::instance()->replaceLine(
      INFO_SCREEN_CLIENTS_LINE, "TCP Conn: %02d"
    , webSocketClients.size() + jmriClients.size());
  }
  else if (type == WS_EVT_DISCONNECT)
  {
    auto const &elem = std::find_if(webSocketClients.begin(), webSocketClients.end(),
      [client](unique_ptr<WebSocketClient> & clientNode) -> bool
      {
        return clientNode->getID() == client->id();
      }
    );
    if (elem != webSocketClients.end())
    {
      webSocketClients.erase(elem);
    }
    Singleton<InfoScreen>::instance()->replaceLine(INFO_SCREEN_CLIENTS_LINE, "TCP Conn: %02d",
                            webSocketClients.size() + jmriClients.size());
  }
  else if (type == WS_EVT_DATA)
  {
    auto const &elem = std::find_if(webSocketClients.begin(), webSocketClients.end(),
      [client](unique_ptr<WebSocketClient> & clientNode) -> bool
      {
        return clientNode->getID() == client->id();
      }
    );
    if (elem != webSocketClients.end())
    {
      string res = elem->get()->feed(data, len);
      if (!res.empty())
      {
        client->text(res.c_str());
      }
    }
  }
}

ESP32CSWebServer::ESP32CSWebServer(MDNS *mdns) : mdns_(mdns)
{
}

#define BUILTIN_URI(uri) \
  webServer.on(uri \
             , HTTP_GET \
             , std::bind(&ESP32CSWebServer::streamResource, this, std::placeholders::_1));

#define GET_URI(uri, method) \
  webServer.on(uri \
             , HTTP_GET \
             , std::bind(&ESP32CSWebServer::method, this, std::placeholders::_1));

#define GET_POST_URI(uri, method) \
  webServer.on(uri \
             , HTTP_GET | HTTP_POST \
             , std::bind(&ESP32CSWebServer::method, this, std::placeholders::_1));

#define GET_PUT_URI(uri, method) \
  webServer.on(uri \
             , HTTP_GET | HTTP_PUT \
             , std::bind(&ESP32CSWebServer::method, this, std::placeholders::_1));

#define GET_POST_DELETE_URI(uri, method) \
  webServer.on(uri \
             , HTTP_GET | HTTP_POST | HTTP_DELETE \
             , std::bind(&ESP32CSWebServer::method, this, std::placeholders::_1));

#define GET_POST_PUT_DELETE_URI(uri, method) \
  webServer.on(uri \
             , HTTP_GET | HTTP_POST | HTTP_PUT | HTTP_DELETE \
             , std::bind(&ESP32CSWebServer::method, this, std::placeholders::_1));

#define POST_UPLOAD_URI(uri, method, callback) \
  webServer.on(uri \
             , HTTP_POST \
             , std::bind(&ESP32CSWebServer::method, this, std::placeholders::_1) \
             , callback);

#define SEND_GENERIC_RESPONSE(request, code) \
  request->send(code); \
  return;

#define SEND_TEXT_RESPONSE(request, code, text) \
  request->send(code, "text/plain", text);

#define SEND_HTML_RESPONSE(request, code, text) \
  request->send(code, "text/html", text);

#define SEND_JSON_RESPONSE(request, response) \
  request->send(STATUS_OK, APPLICATION_JSON_TYPE, response.c_str()); \
  return; \

#define SEND_JSON_IF_OBJECT(request, object) \
  if (object) \
  { \
    SEND_JSON_RESPONSE(request, object->toJson()) \
  } \
  else \
  { \
    SEND_GENERIC_RESPONSE(request, STATUS_NOT_FOUND) \
  }

#define SEND_JSON_IF_OBJECT_ARG(request, object, arg) \
  if (object) \
  { \
    SEND_JSON_RESPONSE(request, object->toJson(arg)) \
  } \
  else \
  { \
    SEND_GENERIC_RESPONSE(request, STATUS_NOT_FOUND) \
  }

esp_ota_handle_t otaHandle;
void otaUploadCallback(AsyncWebServerRequest *request
                     , const String& filename
                     , size_t index
                     , uint8_t *data
                     , size_t len
                     , bool final)
{
  esp_err_t res = ESP_OK;
  if (!index)
  {
    request->_tempObject = (void *)esp_ota_get_next_update_partition(NULL);
    res = esp_ota_begin((esp_partition_t *)request->_tempObject
                      , OTA_SIZE_UNKNOWN
                      , &otaHandle);
    if (res != ESP_OK)
    {
      goto ota_failure;
    }
    LOG(INFO, "[WebSrv] OTA Update starting...");
    trackSignal->disable_ops_output();
    trackSignal->disable_prog_output();
    Singleton<OTAMonitorFlow>::instance()->report_start();
  }
  res = esp_ota_write(otaHandle, data, len);
  if (res != ESP_OK)
  {
    goto ota_failure;
  }
  Singleton<OTAMonitorFlow>::instance()->report_progress(len);
  if (final)
  {
    res = esp_ota_end(otaHandle);
    if (res != ESP_OK)
    {
      goto ota_failure;
    }
    LOG(INFO, "[WebSrv] OTA binary received, setting boot partition");
    res = esp_ota_set_boot_partition((esp_partition_t *)request->_tempObject);
    if (res != ESP_OK)
    {
      goto ota_failure;
    }
    request->_tempObject = nullptr;
    LOG(INFO, "[WebSrv] OTA Update Complete!");
    Singleton<OTAMonitorFlow>::instance()->report_success();
  }
  return;

ota_failure:
  request->_tempObject = nullptr;
  LOG_ERROR("[WebSrv] OTA Update failure: %s (%d)", esp_err_to_name(res), res);
  SEND_TEXT_RESPONSE(request, STATUS_BAD_REQUEST, esp_err_to_name(res))
  Singleton<OTAMonitorFlow>::instance()->report_failure(res);
}

void ESP32CSWebServer::begin()
{
  if (configStore->isAPEnabled())
  {
    LOG(INFO, "[WebSrv] SoftAP mode enabled, starting DNS server");
    tcpip_adapter_ip_info_t ip_info;
    tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_AP, &ip_info);
    // store the IP address for use in the 404 handler
    softAPAddress_ = StringPrintf(IPSTR, IP2STR(&ip_info.ip));
    // start the async dns on the softap address
    asyncDNS.start(53, "*", ip_info.ip);
  }

  BUILTIN_URI("/jquery.min.js")
  BUILTIN_URI("/jquery.mobile-1.5.0-rc1.min.js")
  BUILTIN_URI("/jquery.mobile-1.5.0-rc1.min.css")
  BUILTIN_URI("/jquery.simple.websocket.min.js")
  BUILTIN_URI("/jqClock-lite.min.js")
  BUILTIN_URI("/images/ajax-loader.gif")
  BUILTIN_URI("/index.html")

  GET_URI("/features", handleFeatures)
  GET_POST_URI("/programmer", handleProgrammer)
  GET_PUT_URI("/power", handlePower)
  GET_POST_DELETE_URI("/config", handleConfig)
  GET_POST_PUT_DELETE_URI("/locomotive", handleLocomotive)
  GET_POST_PUT_DELETE_URI("/turnouts", handleTurnouts)
  POST_UPLOAD_URI("/update", handleOTA, otaUploadCallback)

#if ENABLE_OUTPUTS
  GET_POST_PUT_DELETE_URI("/outputs", handleOutputs)
#endif
#if ENABLE_SENSORS
  GET_POST_DELETE_URI("/sensors", handleSensors)
  GET_POST_DELETE_URI("/remoteSensors", handleRemoteSensors)
#if S88_ENABLED
  GET_POST_DELETE_URI("/s88sensors", handleS88Sensors)
#endif
#endif

  webServer.onNotFound(std::bind(&ESP32CSWebServer::notFoundHandler
                               , this
                               , std::placeholders::_1));

  webSocket.onEvent(handleWsEvent);
  webServer.addHandler(&webSocket);
  webServer.begin();
  mdns_->publish("websvr", "_http._tcp", 80);
}

void ESP32CSWebServer::handleProgrammer(AsyncWebServerRequest *request)
{
  if (request->method() == HTTP_GET)
  {
    if (request->arg(JSON_PROG_ON_MAIN).equalsIgnoreCase(JSON_VALUE_TRUE))
    {
      SEND_GENERIC_RESPONSE(request, STATUS_NOT_ALLOWED)
    }
    else if(request->hasArg(JSON_IDENTIFY_NODE))
    {
      int16_t decoderConfig = readCV(CV_NAMES::DECODER_CONFIG);
      uint16_t decoderAddress = 0;
      if (decoderConfig > 0)
      {
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
            SEND_GENERIC_RESPONSE(request, STATUS_SERVER_ERROR)
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
              SEND_GENERIC_RESPONSE(request, STATUS_SERVER_ERROR)
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
              SEND_GENERIC_RESPONSE(request, STATUS_SERVER_ERROR)
            }
          }
          response += StringPrintf("\"%s\":%s,", JSON_SPEED_TABLE_NODE
                                 , bitRead(decoderConfig
                                         , DECODER_CONFIG_BITS::SPEED_TABLE) ?
                                           JSON_VALUE_ON : JSON_VALUE_OFF);
        }
        response += StringPrintf("\"%s\":%d,", JSON_ADDRESS_NODE
                               , decoderAddress);
        bool create = request->arg(JSON_CREATE_NODE).equalsIgnoreCase(JSON_VALUE_TRUE);
        auto roster = locoManager->getRosterEntry(decoderAddress, create);
        if (roster)
        {
          response += StringPrintf("\"%s\":%s,", JSON_LOCO_NODE
                                 , roster->toJson().c_str());
          roster->setType(bitRead(decoderConfig
                                , DECODER_CONFIG_BITS::DECODER_TYPE) ?
                                  JSON_VALUE_STATIONARY_DECODER :
                                  JSON_VALUE_MOBILE_DECODER);
        }
        response += "}";
        SEND_JSON_RESPONSE(request, response)
      }
      else
      {
        LOG(WARNING, "Failed to read decoder configuration");
        SEND_GENERIC_RESPONSE(request, STATUS_SERVER_ERROR)
      }
    }
    else
    {
      uint16_t cvNumber = request->arg(JSON_CV_NODE).toInt();
      int16_t cvValue = readCV(cvNumber);
      if (cvValue < 0)
      {
        SEND_GENERIC_RESPONSE(request, STATUS_SERVER_ERROR)
      }
      SEND_JSON_RESPONSE(request
                       , StringPrintf("{\"%s\":%d,\"%s\":%d}"
                                    , JSON_CV_NODE, cvNumber
                                    , JSON_VALUE_NODE, cvValue))
    }
  }
  else if (request->method() == HTTP_POST && request->hasArg(JSON_PROG_ON_MAIN))
  {
    if (request->arg(JSON_PROG_ON_MAIN).equalsIgnoreCase(JSON_VALUE_TRUE))
    {
      if (request->hasArg(JSON_CV_BIT_NODE))
      {
        writeOpsCVBit(request->arg(JSON_ADDRESS_NODE).toInt(), request->arg(JSON_CV_NODE).toInt(),
          request->arg(JSON_CV_BIT_NODE).toInt(), request->arg(JSON_VALUE_NODE).equalsIgnoreCase(JSON_VALUE_TRUE));
      }
      else
      {
        writeOpsCVByte(request->arg(JSON_ADDRESS_NODE).toInt(), request->arg(JSON_CV_NODE).toInt(),
          request->arg(JSON_VALUE_NODE).toInt());
      }
    }
    else
    {
      if (request->hasArg(JSON_CV_BIT_NODE) &&
          !writeProgCVBit(request->arg(JSON_CV_NODE).toInt(), request->arg(JSON_CV_BIT_NODE).toInt(),
                          request->arg(JSON_VALUE_NODE).equalsIgnoreCase(JSON_VALUE_TRUE)))
      {
        SEND_GENERIC_RESPONSE(request, STATUS_SERVER_ERROR)
      }
      else if (!writeProgCVByte(request->arg(JSON_CV_NODE).toInt(), request->arg(JSON_VALUE_NODE).toInt()))
      {
        SEND_GENERIC_RESPONSE(request, STATUS_SERVER_ERROR)
      }
    }
  }
  SEND_GENERIC_RESPONSE(request, STATUS_OK)
 }

void ESP32CSWebServer::handlePower(AsyncWebServerRequest *request)
{
  if (request->method() == HTTP_GET)
  {
    if (request->params())
    {
      SEND_JSON_RESPONSE(request
                       , StringPrintf("{\"%s\":\"%s\"}"
                                    , JSON_STATE_NODE
                                    , trackSignal->is_enabled() ?
                                        JSON_VALUE_TRUE : JSON_VALUE_FALSE));
    }
    else
    {
      SEND_JSON_RESPONSE(request, trackSignal->generate_status_json())
    }
    return;
  }
  else if (request->method() == HTTP_PUT)
  {
    if (request->arg(JSON_STATE_NODE).equalsIgnoreCase(JSON_VALUE_TRUE))
    {
      trackSignal->enable_ops_output();
    }
    else
    {
      trackSignal->disable_ops_output();
      trackSignal->disable_prog_output();
    }
    SEND_GENERIC_RESPONSE(request, STATUS_OK)
    return;
  }
  SEND_GENERIC_RESPONSE(request, STATUS_BAD_REQUEST)
 }

void ESP32CSWebServer::handleOutputs(AsyncWebServerRequest *request)
{
#if ENABLE_OUTPUTS
  if (request->method() == HTTP_GET && !request->hasArg(JSON_ID_NODE))
  {
    SEND_JSON_RESPONSE(request, OutputManager::getStateAsJson())
  }
  else if (request->method() == HTTP_GET)
  {
    auto output = OutputManager::getOutput(request->arg(JSON_ID_NODE).toInt());
    SEND_JSON_IF_OBJECT_ARG(request, output, true)
  }
  else if (request->method() == HTTP_POST)
  {
    uint16_t outputID = request->arg(JSON_ID_NODE).toInt();
    uint8_t pin = request->arg(JSON_PIN_NODE).toInt();
    bool inverted = request->arg(JSON_INVERTED_NODE).equalsIgnoreCase(JSON_VALUE_TRUE);
    bool forceState = request->arg(JSON_FORCE_STATE_NODE).equalsIgnoreCase(JSON_VALUE_TRUE);
    bool defaultState = request->arg(JSON_DEFAULT_STATE_NODE).equalsIgnoreCase(JSON_VALUE_TRUE);
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
    if (!OutputManager::createOrUpdate(outputID, pin, outputFlags))
    {
      SEND_GENERIC_RESPONSE(request, STATUS_NOT_ALLOWED)
    }
  }
  else if (request->method() == HTTP_DELETE &&
          !OutputManager::remove(request->arg(JSON_ID_NODE).toInt()))
  {
    SEND_GENERIC_RESPONSE(request, STATUS_NOT_FOUND)
  }
  else if(request->method() == HTTP_PUT)
  {
   OutputManager::toggle(request->arg(JSON_ID_NODE).toInt());
  }
  SEND_GENERIC_RESPONSE(request, STATUS_OK)
#endif // ENABLE_OUTPUTS
}

void ESP32CSWebServer::handleTurnouts(AsyncWebServerRequest *request)
{
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

  if (request->method() == HTTP_GET && !request->hasArg(JSON_ID_NODE))
  {
    bool readableStrings = true;
    if (request->hasArg(JSON_TURNOUTS_READABLE_STRINGS_NODE))
    {
      readableStrings = request->arg(JSON_TURNOUTS_READABLE_STRINGS_NODE).toInt();
    }
    SEND_JSON_RESPONSE(request, turnoutManager->getStateAsJson(readableStrings))
  }
  else if (request->method() == HTTP_GET)
  {
    if(request->hasArg(JSON_ID_NODE))
    {
      auto turnout = turnoutManager->getTurnoutByID(request->arg(JSON_ID_NODE).toInt());
      SEND_JSON_IF_OBJECT(request, turnout)
    }
    else if (request->hasArg(JSON_ADDRESS_NODE))
    {
      auto turnout = turnoutManager->getTurnoutByID(request->arg(JSON_ADDRESS_NODE).toInt());
      SEND_JSON_IF_OBJECT(request, turnout)
    }
    else
    {
      SEND_GENERIC_RESPONSE(request, STATUS_BAD_REQUEST)
    }
  }
  else if (request->method() == HTTP_POST)
  {
    int32_t turnoutID = request->arg(JSON_ID_NODE).toInt();
    uint16_t turnoutAddress = request->arg(JSON_ADDRESS_NODE).toInt();
    int8_t turnoutSubAddress = request->arg(JSON_SUB_ADDRESS_NODE).toInt();
    TurnoutType type = (TurnoutType)request->arg(JSON_TYPE_NODE).toInt();
    // auto generate ID
    if (turnoutID == -1)
    {
      turnoutID = turnoutManager->getTurnoutCount() + 1;
    }
    auto turnout = turnoutManager->createOrUpdate((uint16_t)turnoutID, turnoutAddress, turnoutSubAddress, type);
    SEND_JSON_IF_OBJECT(request, turnout)
  }
  else if (request->method() == HTTP_DELETE)
  {
    if (request->hasArg(JSON_ID_NODE))
    {
      turnoutManager->removeByID(request->arg(JSON_ID_NODE).toInt());
    }
    else if (request->hasArg(JSON_ADDRESS_NODE))
    {
      turnoutManager->removeByAddress(request->arg(JSON_ADDRESS_NODE).toInt());
    }
    else
    {
      SEND_GENERIC_RESPONSE(request, STATUS_BAD_REQUEST)
    }
  }
  else if (request->method() == HTTP_PUT)
  {
    if (request->hasArg(JSON_ID_NODE))
    {
      turnoutManager->toggleByID(request->arg(JSON_ID_NODE).toInt());
    }
    else if (request->hasArg(JSON_ADDRESS_NODE))
    {
      turnoutManager->toggleByAddress(request->arg(JSON_ADDRESS_NODE).toInt());
    }
    else
    {
      SEND_GENERIC_RESPONSE(request, STATUS_BAD_REQUEST)
    }
  }
  SEND_GENERIC_RESPONSE(request, STATUS_OK)
}

void ESP32CSWebServer::handleSensors(AsyncWebServerRequest *request)
{
#if ENABLE_SENSORS
  if (request->method() == HTTP_GET && !request->hasArg(JSON_ID_NODE))
  {
    SEND_JSON_RESPONSE(request, SensorManager::getStateAsJson())
    return;
  }
  else if (request->method() == HTTP_GET)
  {
    auto sensor = SensorManager::getSensor(request->arg(JSON_ID_NODE).toInt());
    SEND_JSON_IF_OBJECT(request, sensor)
  }
  else if(request->method() == HTTP_POST)
  {
    uint16_t sensorID = request->arg(JSON_ID_NODE).toInt();
    uint8_t sensorPin = request->arg(JSON_PIN_NODE).toInt();
    bool sensorPullUp = request->arg(JSON_PULLUP_NODE).equalsIgnoreCase(JSON_VALUE_TRUE);
    if(!SensorManager::createOrUpdate(sensorID, sensorPin, sensorPullUp))
    {
      SEND_GENERIC_RESPONSE(request, STATUS_NOT_ALLOWED)
    }
  }
  else if(request->method() == HTTP_DELETE)
  {
    uint16_t sensorID = request->arg(JSON_ID_NODE).toInt();
    if(SensorManager::getSensorPin(sensorID) < 0)
    {
      // attempt to delete S88/RemoteSensor
      SEND_GENERIC_RESPONSE(request, STATUS_NOT_ALLOWED)
    }
    else if(!SensorManager::remove(sensorID))
    {
      SEND_GENERIC_RESPONSE(request, STATUS_NOT_FOUND)
    }
  }
  SEND_GENERIC_RESPONSE(request, STATUS_OK)
#endif // ENABLE_SENSORS
}

void ESP32CSWebServer::handleConfig(AsyncWebServerRequest *request)
{
  if (request->method() == HTTP_POST)
  {
    DCCPPProtocolHandler::getCommandHandler("E")->process(vector<string>());
  }
  else if (request->method() == HTTP_DELETE)
  {
    DCCPPProtocolHandler::getCommandHandler("e")->process(vector<string>());
  }
  if (request->hasArg("scan"))
  {
    SyncNotifiable n;
    wifiManager->start_ssid_scan(&n);
    n.wait_for_notification();
    size_t num_found = wifiManager->get_ssid_scan_result_count();
    nlohmann::json ssid_list;
    for (int i = 0; i < num_found; i++)
    {
      auto result = wifiManager->get_ssid_scan_result(i);
      ssid_list.push_back(
        {
          {"ssid", string((char *)result.ssid)},
          {"rssi", result.rssi},
          {"auth", result.authmode}
        }
      );
    }
    wifiManager->clear_ssid_scan_results();
    SEND_JSON_RESPONSE(request, ssid_list.dump())
  }
  else if (request->hasArg("ssid"))
  {
    LOG(INFO, "Selected SSID: %s / %s", request->arg("ssid").c_str()
      , request->arg("password").c_str());
  }
  SEND_JSON_RESPONSE(request, configStore->getCSConfig())
}

#if S88_ENABLED && ENABLE_SENSORS
void ESP32CSWebServer::handleS88Sensors(AsyncWebServerRequest *request)
{
  if(request->method() == HTTP_GET)
  {
    SEND_JSON_RESPONSE(request, S88BusManager::getStateAsJson())
  }
  else if(request->method() == HTTP_POST)
  {
    if(!S88BusManager::createOrUpdateBus(
      request->arg(JSON_ID_NODE).toInt(),
      request->arg(JSON_PIN_NODE).toInt(),
      request->arg(JSON_COUNT_NODE).toInt()))
    {
      // duplicate pin/id
      SEND_GENERIC_RESPONSE(request, STATUS_NOT_ALLOWED)
    }
  }
  else if(request->method() == HTTP_DELETE)
  {
    S88BusManager::removeBus(request->arg(JSON_ID_NODE).toInt());
  }
  SEND_GENERIC_RESPONSE(request, STATUS_OK)
}
#endif

void ESP32CSWebServer::handleRemoteSensors(AsyncWebServerRequest *request) {
#if ENABLE_SENSORS
  if(request->method() == HTTP_GET)
  {
    SEND_JSON_RESPONSE(request, RemoteSensorManager::getStateAsJson())
  }
  else if(request->method() == HTTP_POST)
  {
    RemoteSensorManager::createOrUpdate(request->arg(JSON_ID_NODE).toInt(),
      request->arg(JSON_VALUE_NODE).toInt());
  }
  else if(request->method() == HTTP_DELETE)
  {
    RemoteSensorManager::remove(request->arg(JSON_ID_NODE).toInt());
  }
  SEND_GENERIC_RESPONSE(request, STATUS_OK)
#endif // ENABLE_SENSORS
}

void ESP32CSWebServer::handleLocomotive(AsyncWebServerRequest *request)
{
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
  string url(request->url().c_str());
  // check if we have an eStop command, we don't care how this gets sent to the
  // command station (method) so check it first
  if(url.find("/estop") != string::npos)
  {
    locoManager->set_state(true);
  }
  else if(url.find("/roster")  != string::npos)
  {
    if(request->method() == HTTP_GET && !request->hasArg(JSON_ADDRESS_NODE))
    {
      if(request->hasArg("new"))
      {
        SEND_JSON_RESPONSE(request,
                           Singleton<esp32cs::Esp32TrainDatabase>::instance()->get_train_list_as_json())
      }
      SEND_JSON_RESPONSE(request, locoManager->getRosterEntriesAsJson())
    }
    else if (request->hasArg(JSON_ADDRESS_NODE))
    {
      if(request->method() == HTTP_DELETE)
      {
        locoManager->removeRosterEntry(request->arg(JSON_ADDRESS_NODE).toInt());
        SEND_GENERIC_RESPONSE(request, STATUS_OK)
      }
      else
      {
        RosterEntry *entry = locoManager->getRosterEntry(request->arg(JSON_ADDRESS_NODE).toInt());
        if(request->method() == HTTP_PUT || request->method() == HTTP_POST)
        {
          if(request->hasArg(JSON_DESCRIPTION_NODE))
          {
            entry->setDescription(request->arg(JSON_DESCRIPTION_NODE).c_str());
          }
          if(request->hasArg(JSON_TYPE_NODE))
          {
            entry->setType(request->arg(JSON_TYPE_NODE).c_str());
          }
          if(request->hasArg(JSON_IDLE_ON_STARTUP_NODE))
          {
            entry->setIdleOnStartup(request->arg(JSON_IDLE_ON_STARTUP_NODE).equalsIgnoreCase(JSON_VALUE_TRUE));
          }
          if(request->hasArg(JSON_DEFAULT_ON_THROTTLE_NODE))
          {
            entry->setDefaultOnThrottles(request->arg(JSON_DEFAULT_ON_THROTTLE_NODE).equalsIgnoreCase(JSON_VALUE_TRUE));
          }
        }
        SEND_JSON_IF_OBJECT(request, entry)
      }
    }
  }
  else
  {
    // Since it is not an eStop or roster command we need to check the request
    // method and ensure it contains the required arguments otherwise the
    // request should be rejected
    if(request->method() == HTTP_GET && !request->hasArg(JSON_ADDRESS_NODE))
    {
      // get all active locomotives
      SEND_JSON_RESPONSE(request, locoManager->getActiveLocosAsJson())
    }
    else if (request->hasArg(JSON_ADDRESS_NODE))
    {
      if(request->method() == HTTP_PUT || request->method() == HTTP_POST)
      {
        auto loco = locoManager->getLocomotive(request->arg(JSON_ADDRESS_NODE).toInt());
        auto upd_speed = loco->get_speed();
        // Creation / Update of active locomotive
        if(request->hasArg(JSON_IDLE_NODE) && request->arg(JSON_IDLE_NODE).equalsIgnoreCase(JSON_VALUE_TRUE))
        {
          upd_speed.set_dcc_128(0);
        }
        if(request->hasArg(JSON_SPEED_NODE))
        {
          upd_speed.set_dcc_128(request->arg(JSON_SPEED_NODE).toInt());
        }
        if(request->hasArg(JSON_DIRECTION_NODE))
        {
          upd_speed.set_direction(request->arg(JSON_DIRECTION_NODE).equalsIgnoreCase(JSON_VALUE_FORWARD) ? dcc::SpeedType::FORWARD : dcc::SpeedType::REVERSE);
        }
        loco->set_speed(upd_speed);
        for(uint8_t funcID = 0; funcID <=28 ; funcID++)
        {
          string fArg = StringPrintf("f%d", funcID);
          if(request->hasArg(fArg.c_str()))
          {
            loco->set_fn(funcID, request->arg(fArg.c_str()).equalsIgnoreCase(JSON_VALUE_TRUE));
          }
        }
        SEND_JSON_IF_OBJECT(request, loco)
      }
      else if(request->method() == HTTP_DELETE)
      {
        // Removal of an active locomotive
        locoManager->removeLocomotive(request->arg(JSON_ADDRESS_NODE).toInt());
#if NEXTION_ENABLED
        static_cast<NextionThrottlePage *>(nextionPages[THROTTLE_PAGE])->invalidateLocomotive(request->arg(JSON_ADDRESS_NODE).toInt());
#endif
        SEND_GENERIC_RESPONSE(request, STATUS_OK)
      }
      else
      {
        auto loco = locoManager->getLocomotive(request->arg(JSON_ADDRESS_NODE).toInt());
        SEND_JSON_IF_OBJECT(request, loco)
      }
    }
  }
  // missing arg or unknown request
  SEND_GENERIC_RESPONSE(request, STATUS_BAD_REQUEST)
}

void ESP32CSWebServer::handleOTA(AsyncWebServerRequest *request)
{
  SEND_TEXT_RESPONSE(request, STATUS_OK, "OTA Upload Complete")
}

void ESP32CSWebServer::handleFeatures(AsyncWebServerRequest *request)
{
  SEND_JSON_RESPONSE(request, configStore->getCSFeatures())
}

#define IP_TO_STR(request) \
  ipv4_to_string(ntohl(request->client()->getRemoteAddress())).c_str()

#define STREAM_RESOURCE_NO_REDIRECT(request, uri, mimeType, totalSize, resource) \
  if (request->url() == uri) \
  { \
    LOG(INFO, "[WebSrv %s] Sending %s from MEMORY (%d, %s)", IP_TO_STR(request), uri, totalSize, mimeType); \
    response = request->beginResponse_P(STATUS_OK, mimeType, resource, totalSize); \
    if(!string(mimeType).compare(0, 5, "text/")) \
    { \
      response->addHeader("Content-Encoding", "gzip"); \
    } \
  }

#define STREAM_RESOURCE(request, uri, fallback, mimeType, totalSize, resource) \
  if (request->url() == uri && configStore->isAPEnabled() && softAPAddress_.compare(request->host().c_str()) == 0) \
  { \
    LOG(INFO, "[WebSrv %s] Sending %s from MEMORY (%d, %s)", IP_TO_STR(request), uri, totalSize, mimeType); \
    response = request->beginResponse_P(STATUS_OK, mimeType, resource, totalSize); \
    if(!string(mimeType).compare(0, 5, "text/")) \
    { \
      response->addHeader("Content-Encoding", "gzip"); \
    } \
  } \
  else if (request->url() == uri) \
  { \
    LOG(INFO, "[WebSrv %s] Requested %s => CDN %s", IP_TO_STR(request), uri, fallback); \
    request->redirect(fallback); \
  }

void ESP32CSWebServer::streamResource(AsyncWebServerRequest *request)
{
  const char * htmlBuildTime = __DATE__ " " __TIME__;
  if (request->header("If-Modified-Since").equals(htmlBuildTime))
  {
    SEND_GENERIC_RESPONSE(request, STATUS_NOT_MODIFIED)
  }
  else
  {
    AsyncWebServerResponse *response = nullptr;
    STREAM_RESOURCE_NO_REDIRECT(request, "/index.html", "text/html"
                              , indexHtmlGz_size, indexHtmlGz)
    STREAM_RESOURCE(request, "/jquery.min.js"
                  , "https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js"
                  , "text/javascript", jqueryJsGz_size, jqueryJsGz)
    STREAM_RESOURCE(request, "/jquery.mobile-1.5.0-rc1.min.js"
                  , "https://code.jquery.com/mobile/1.5.0-rc1/jquery.mobile-1.5.0-rc1.min.js"
                  , "text/javascript", jqueryMobileJsGz_size, jqueryMobileJsGz)
    STREAM_RESOURCE(request, "/jquery.mobile-1.5.0-rc1.min.css"
                  , "https://code.jquery.com/mobile/1.5.0-rc1/jquery.mobile-1.5.0-rc1.min.css"
                  , "text/css", jqueryMobileCssGz_size, jqueryMobileCssGz)
    STREAM_RESOURCE(request, "/jquery.simple.websocket.min.js"
                  , "https://cdn.rawgit.com/jbloemendal/jquery-simple-websocket/master/dist/jquery.simple.websocket.min.js"
                  , "text/javascript", jquerySimpleWebSocketGz_size, jquerySimpleWebSocketGz)
    STREAM_RESOURCE(request, "/jqClock-lite.min.js"
                  , "https://cdn.rawgit.com/JohnRDOrazio/jQuery-Clock-Plugin/master/jqClock-lite.min.js"
                  , "text/javascript", jqClockGz_size, jqClockGz)
    STREAM_RESOURCE(request, "/images/ajax-loader.gif"
                  , "https://code.jquery.com/mobile/1.5.0-rc1/images/ajax-loader.gif"
                  , "image/gif", ajaxLoader_size, ajaxLoader)
    if (response)
    {
      response->addHeader("Last-Modified", htmlBuildTime);
      SEND_GENERIC_RESPONSE(request, response)
    }
    else
    {
      SEND_GENERIC_RESPONSE(request, STATUS_NOT_FOUND)
    }
  }
}

// Known captive portal checks to respond with http 200 and redirect link
static string portalCheckRedirect[] =
{
  "/generate_204",                  // Android
  "/gen_204",                       // Android 9.0
  "/mobile/status.php",             // Android 8.0 (Samsung s9+)
  "/ncsi.txt",                      // Windows
  "/success.txt",                   // OSX / FireFox
  "/hotspot-detect.html",           // iOS 8/9
  "/hotspotdetect.html",            // iOS 8/9
  "/library/test/success.html"      // iOS 8/9
  "/kindle-wifi/wifiredirect.html"  // Kindle
  "/kindle-wifi/wifistub.html"      // Kindle
};

// Captive Portal landing page
static constexpr const char * const CAPTIVE_PORTAL_HTML =
  "<html>"
  "<title>ESP32 Command Station v%s</title>"
  "<meta http-equiv=\"refresh\" content=\"30;url='http://%s/captiveauth'\" /> "
  "<body>"
  "<h1>Welcome to the ESP32 Command Station</h1>"
  "<h2>Open your browser and navigate to any website and it will open the "
  "ESP32 Command Station</h2>"
  "<p>Click <a href=\"http://%s/captiveauth\">here</a> to close this portal "
  "page if it does not automatically close.</p>" 
  "</body>"
  "</html>";

// iOS success page content
static constexpr const char * const CAPTIVE_SUCCESS_200_HTML = 
  "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>";
// Windows success page content
static constexpr const char * const CAPTIVE_SUCCESS_200_NCSI_TXT = 
  "Microsoft NCSI";
// Generic success.txt page content
static constexpr const char * const CAPTIVE_SUCCESS_200_SUCCESS_TXT = 
  "success";

void ESP32CSWebServer::notFoundHandler(AsyncWebServerRequest *request)
{
  if (configStore->isAPEnabled())
  {
    uint32_t clientIP = static_cast<uint32_t>(request->client()->remoteIP());
    for (auto uri : portalCheckRedirect)
    {
      if (!uri.compare(request->url().c_str()))
      {
        if (std::find(captiveIPs_.begin(), captiveIPs_.end()
                    , clientIP) != captiveIPs_.end())
        {
          // we have seen this IP previously and they have clicked through
          // to the index.html already, send back a 200 or 204 response.
          if (request->url().endsWith("_204") ||
              request->url().endsWith("status.php"))
          {
            // no payload required for this one, just the status code.
            SEND_GENERIC_RESPONSE(request, STATUS_NO_CONTENT)
          }
          else if (request->url().endsWith("ncsi.txt"))
          {
            SEND_TEXT_RESPONSE(request, STATUS_OK, CAPTIVE_SUCCESS_200_NCSI_TXT)
          }
          else if (request->url().endsWith("success.txt"))
          {
            SEND_TEXT_RESPONSE(request, STATUS_OK, CAPTIVE_SUCCESS_200_SUCCESS_TXT)
          }
          else
          {
            SEND_TEXT_RESPONSE(request, STATUS_OK, CAPTIVE_SUCCESS_200_HTML)
          }
        }
        else
        {
          LOG(INFO, "[WebSrv %s] Requested %s%s: Redirecting to captive portal"
            , IP_TO_STR(request), request->host().c_str()
            , request->url().c_str());
          SEND_HTML_RESPONSE(request, STATUS_OK,
                        StringPrintf(CAPTIVE_PORTAL_HTML
                                   , ESP32CS_VERSION
                                   , softAPAddress_.c_str()
                                   , softAPAddress_.c_str()
                        ).c_str())
        }
        return;
      }
    }
    if (request->url().equals("/captiveauth"))
    {
      // check if we have seen this client IP before (for captive portal check)
      if (std::find(captiveIPs_.begin(), captiveIPs_.end()
                  , clientIP) == captiveIPs_.end())
      {
        captiveIPs_.push_back(clientIP);
      }
      SEND_GENERIC_RESPONSE(request, STATUS_OK)
    }
  }
  if (request->url().equals("/"))
  {
    string interfaceIP(request->client()->localIP().toString().c_str());
    request->redirect(StringPrintf("http://%s/index.html", interfaceIP.c_str()).c_str());
    return;
  }
  LOG(INFO, "[WebSrv %s] 404: %s%s", IP_TO_STR(request)
    , request->host().c_str(), request->url().c_str());
  SEND_GENERIC_RESPONSE(request, STATUS_NOT_FOUND)
}