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
#include <AsyncJson.h>

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

enum HTTP_STATUS_CODES {
  STATUS_OK = 200,
  STATUS_NOT_MODIFIED = 304,
  STATUS_BAD_REQUEST = 400,
  STATUS_NOT_FOUND = 404,
  STATUS_NOT_ALLOWED = 405,
  STATUS_NOT_ACCEPTABLE = 406,
  STATUS_CONFLICT = 409,
  STATUS_PRECONDITION_FAILED = 412,
  STATUS_SERVER_ERROR = 500
};

Atomic webSocketAtomic;
std::vector<WebSocketClient *> webSocketClients;

void handleWsEvent(AsyncWebSocket * server,
                   AsyncWebSocketClient * client,
                   AwsEventType type,
                   void * arg,
                   uint8_t *data,
                   size_t len) {
  if (type == WS_EVT_CONNECT) {
    {
      AtomicHolder h(&webSocketAtomic);
      webSocketClients.push_back(new WebSocketClient(client->id(), ntohl(client->remoteIP())));
    }
    client->printf("<iDCC++ ESP32 Command Station: V-%s / %s %s>", VERSION, __DATE__, __TIME__);
    infoScreen->replaceLine(INFO_SCREEN_CLIENTS_LINE, 
                            "TCP Conn: %02d",
                            webSocketClients.size() + jmriClients.size());
  } else if (type == WS_EVT_DISCONNECT) {
    {
      AtomicHolder h(&webSocketAtomic);
      auto elem = std::find_if(webSocketClients.begin(), webSocketClients.end(),
        [client](WebSocketClient *clientNode) -> bool {
          return (clientNode->getID() == client->id());
        }
      );
      if(elem != webSocketClients.end()) {
        delete *elem;
        webSocketClients.erase(elem);
      }
    }
    infoScreen->replaceLine(INFO_SCREEN_CLIENTS_LINE,
                            "TCP Conn: %02d",
                            webSocketClients.size() + jmriClients.size());
  } else if (type == WS_EVT_DATA) {
    {
      AtomicHolder h(&webSocketAtomic);
      for (auto clientNode : webSocketClients) {
        if(clientNode->getID() == client->id()) {
          clientNode->feed(data, len);
        }
      }
    }
  }
}

ESP32CSWebServer::ESP32CSWebServer(MDNS *mdns) : mdns_(mdns) {
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
             , HTTP_GET | HTTP_POST \
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
    res = esp_ota_begin(esp_ota_get_next_update_partition(NULL)
                      , OTA_SIZE_UNKNOWN
                      , &otaHandle);
    if (res != ESP_OK)
    {
      goto ota_failure;
    }
    LOG(INFO, "[WebSrv] OTA Update starting...");
    disable_all_hbridges();
    otaMonitor->report_start();
  }
  res = esp_ota_write(otaHandle, data, len);
  if (res != ESP_OK)
  {
    goto ota_failure;
  }
  otaMonitor->report_progress(len);
  if (final)
  {
    res = esp_ota_end(otaHandle);
    if (res != ESP_OK)
    {
      goto ota_failure;
    }
    LOG(INFO, "[WebSrv] OTA Update Complete!");
    otaMonitor->report_success();
  }
  return;

ota_failure:
  LOG_ERROR("[WebSrv] OTA Update failure: %s (%d)", esp_err_to_name(res), res);
  request->send(STATUS_BAD_REQUEST, "text/plain", esp_err_to_name(res));
  otaMonitor->report_failure(res);
}

void ESP32CSWebServer::begin() {
  if (configStore->isAPEnabled())
  {
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
  GET_POST_PUT_DELETE_URI("/outputs", handleOutputs)
  GET_POST_DELETE_URI("/sensors", handleSensors)
  GET_POST_DELETE_URI("/remoteSensors", handleRemoteSensors)
  GET_POST_DELETE_URI("/config", handleConfig)
  GET_POST_PUT_DELETE_URI("/locomotive", handleLocomotive)
  POST_UPLOAD_URI("/update", handleOTA, otaUploadCallback)

#if S88_ENABLED
  GET_POST_DELETE_URI("/s88sensors", handleS88Sensors)
#endif

  webServer.rewrite("/", "/index.html");
  webServer.onNotFound(std::bind(&ESP32CSWebServer::notFoundHandler
                               , this
                               , std::placeholders::_1));

  webSocket.onEvent(handleWsEvent);
  webServer.addHandler(&webSocket);
  webServer.begin();
  mdns_->publish("websvr", "_http._tcp", 80);
}

void ESP32CSWebServer::broadcastToWS(const std::string &buf) {
  webSocket.textAll(buf.c_str());
}

void ESP32CSWebServer::handleProgrammer(AsyncWebServerRequest *request) {
  auto jsonResponse = new AsyncJsonResponse();
  // new programmer request
  if (request->method() == HTTP_GET) {
    if (request->arg(JSON_PROG_ON_MAIN).equalsIgnoreCase(JSON_VALUE_TRUE)) {
      jsonResponse->setCode(STATUS_NOT_ALLOWED);
    } else if(request->hasArg(JSON_IDENTIFY_NODE)) {
      JsonObject node = jsonResponse->getRoot();
      if(enterProgrammingMode()) {
        int16_t decoderConfig = readCV(CV_NAMES::DECODER_CONFIG);
        uint16_t decoderAddress = 0;
        if(decoderConfig > 0) {
          if(bitRead(decoderConfig, DECODER_CONFIG_BITS::DECODER_TYPE)) {
            uint8_t decoderManufacturer = readCV(CV_NAMES::DECODER_MANUFACTURER);
            int16_t addrMSB = readCV(CV_NAMES::ACCESSORY_DECODER_MSB_ADDRESS);
            int16_t addrLSB = readCV(CV_NAMES::SHORT_ADDRESS);
            if(addrMSB >= 0 && addrLSB >= 0) {
              if(decoderManufacturer == 0xA5) { // MERG uses 7 bit LSB
                decoderAddress = (uint16_t)(((addrMSB & 0x07) << 7) | (addrLSB & 0x7F));
              } else if(decoderManufacturer == 0x19) { // Team Digital uses 8 bit LSB and 4 bit MSB
                decoderAddress = (uint16_t)(((addrMSB & 0x0F) << 8) | addrLSB);
              } else { // NMRA spec shows 6 bit LSB
                decoderAddress = (uint16_t)(((addrMSB & 0x07) << 6) | (addrLSB & 0x1F));
              }
              node[JSON_ADDRESS_MODE_NODE] = JSON_VALUE_LONG_ADDRESS;
            } else {
              LOG(WARNING, "Failed to read address MSB/LSB");
              jsonResponse->setCode(STATUS_SERVER_ERROR);
            }
          } else {
            if(bitRead(decoderConfig, DECODER_CONFIG_BITS::SHORT_OR_LONG_ADDRESS)) {
              int16_t addrMSB = readCV(CV_NAMES::LONG_ADDRESS_MSB_ADDRESS);
              int16_t addrLSB = readCV(CV_NAMES::LONG_ADDRESS_LSB_ADDRESS);
              if(addrMSB >= 0 && addrLSB >= 0) {
                decoderAddress = (uint16_t)(((addrMSB & 0xFF) << 8) | (addrLSB & 0xFF));
                node[JSON_ADDRESS_MODE_NODE] = JSON_VALUE_LONG_ADDRESS;
              } else {
                LOG(WARNING, "Unable to read address MSB/LSB");
                jsonResponse->setCode(STATUS_SERVER_ERROR);
              }
            } else {
              int16_t shortAddr = readCV(CV_NAMES::SHORT_ADDRESS);
              if(shortAddr > 0) {
                decoderAddress = shortAddr;
                node[JSON_ADDRESS_MODE_NODE] = JSON_VALUE_SHORT_ADDRESS;
              } else {
                LOG(WARNING, "Unable to read short address CV");
                jsonResponse->setCode(STATUS_SERVER_ERROR);
              }
            }
            if(bitRead(decoderConfig, DECODER_CONFIG_BITS::SPEED_TABLE)) {
              node[JSON_SPEED_TABLE_NODE] = JSON_VALUE_ON;
            } else {
              node[JSON_SPEED_TABLE_NODE] = JSON_VALUE_OFF;
            }
          }
          if(decoderAddress > 0) {
            node[JSON_ADDRESS_NODE] = decoderAddress;
            auto roster = LocomotiveManager::getRosterEntry(decoderAddress, false);
            if(roster) {
              node[JSON_LOCO_NODE] = roster;
            } else if(request->hasArg(JSON_CREATE_NODE) && request->arg(JSON_CREATE_NODE).equalsIgnoreCase(JSON_VALUE_TRUE)) {
              roster = LocomotiveManager::getRosterEntry(decoderAddress);
              if(decoderConfig > 0) {
                if(bitRead(decoderConfig, DECODER_CONFIG_BITS::DECODER_TYPE)) {
                  roster->setType(JSON_VALUE_STATIONARY_DECODER);
                } else {
                  roster->setType(JSON_VALUE_MOBILE_DECODER);
                }
              }
              node[JSON_LOCO_NODE] = roster;
            }
          } else {
            LOG(WARNING, "Failed to read decoder address");
            jsonResponse->setCode(STATUS_SERVER_ERROR);
          }
        } else {
          LOG(WARNING, "Failed to read decoder configuration");
          jsonResponse->setCode(STATUS_SERVER_ERROR);
        }
        leaveProgrammingMode();
      } else {
        LOG(WARNING, "Programmer already in use");
        jsonResponse->setCode(STATUS_SERVER_ERROR);
      }
    } else {
      uint16_t cvNumber = request->arg(JSON_CV_NODE).toInt();
      if(enterProgrammingMode()) {
        int16_t cvValue = readCV(cvNumber);
        JsonObject node = jsonResponse->getRoot();
        node[JSON_CV_NODE] = cvNumber;
        node[JSON_VALUE_NODE] = cvValue;
        if(cvValue < 0) {
          jsonResponse->setCode(STATUS_SERVER_ERROR);
        } else {
          jsonResponse->setCode(STATUS_OK);
        }
        leaveProgrammingMode();
      } else {
        LOG(WARNING, "Programmer already in use");
        jsonResponse->setCode(STATUS_SERVER_ERROR);
      }
    }
  } else if(request->method() == HTTP_POST && request->hasArg(JSON_PROG_ON_MAIN)) {
    if (request->arg(JSON_PROG_ON_MAIN).equalsIgnoreCase(JSON_VALUE_TRUE)) {
      if(request->hasArg(JSON_CV_BIT_NODE)) {
        writeOpsCVBit(request->arg(JSON_ADDRESS_NODE).toInt(), request->arg(JSON_CV_NODE).toInt(),
          request->arg(JSON_CV_BIT_NODE).toInt(), request->arg(JSON_VALUE_NODE).equalsIgnoreCase(JSON_VALUE_TRUE));
      } else {
        writeOpsCVByte(request->arg(JSON_ADDRESS_NODE).toInt(), request->arg(JSON_CV_NODE).toInt(),
          request->arg(JSON_VALUE_NODE).toInt());
      }
      jsonResponse->setCode(STATUS_OK);
    } else {
      bool writeSuccess = false;
      if(enterProgrammingMode()) {
        if(request->hasArg(JSON_CV_BIT_NODE)) {
          writeSuccess = writeProgCVBit(request->arg(JSON_CV_NODE).toInt(), request->arg(JSON_CV_BIT_NODE).toInt(),
            request->arg(JSON_VALUE_NODE).equalsIgnoreCase(JSON_VALUE_TRUE));
        } else {
          writeSuccess = writeProgCVByte(request->arg(JSON_CV_NODE).toInt(), request->arg(JSON_VALUE_NODE).toInt());
        }
        leaveProgrammingMode();
      }
      if(writeSuccess) {
        jsonResponse->setCode(STATUS_OK);
      } else {
        jsonResponse->setCode(STATUS_SERVER_ERROR);
      }
    }
  } else {
    jsonResponse->setCode(STATUS_BAD_REQUEST);
  }
  jsonResponse->setLength();
  request->send(jsonResponse);
 }

void ESP32CSWebServer::handlePower(AsyncWebServerRequest *request)
{
  if(request->method() == HTTP_GET)
  {
    AsyncJsonResponse *jsonResponse = nullptr;
    if(request->params())
    {
      jsonResponse = new AsyncJsonResponse();
      if (is_track_power_on())
      {
        jsonResponse->getRoot()[JSON_STATE_NODE] = JSON_VALUE_TRUE;
      }
      else
      {
        jsonResponse->getRoot()[JSON_STATE_NODE] = JSON_VALUE_FALSE;
      }
    }
    else
    {
      jsonResponse = new AsyncJsonResponse(true);
      get_hbridge_status_json(jsonResponse->getRoot());
    }
    jsonResponse->setLength();
    request->send(jsonResponse);
    return;
  }
  else if (request->method() == HTTP_PUT)
  {
    if(request->hasArg(JSON_OVERALL_STATE_NODE))
    {
      if(request->arg(JSON_OVERALL_STATE_NODE).equalsIgnoreCase(JSON_VALUE_TRUE))
      {
        enable_all_hbridges();
      }
      else
      {
        disable_all_hbridges();
      }
    }
    else if(request->hasArg(JSON_NAME_NODE))
    {
      string bridge = request->arg(JSON_NAME_NODE).c_str();
      if (request->arg(JSON_STATE_NODE).equalsIgnoreCase(JSON_VALUE_TRUE))
      {
        enable_named_hbridge(bridge);
      }
      else
      {
        disable_named_hbridge(bridge);
      }
    }
    request->send(STATUS_OK);
    return;
  }
  request->send(STATUS_BAD_REQUEST);
 }

void ESP32CSWebServer::handleOutputs(AsyncWebServerRequest *request) {
  auto jsonResponse = new AsyncJsonResponse(request->method() == HTTP_GET && !request->params());
  if (request->method() == HTTP_GET && !request->hasArg(JSON_ID_NODE)) {
    JsonArray array = jsonResponse->getRoot();
    OutputManager::getState(array);
  } else if (request->method() == HTTP_GET) {
    auto output = OutputManager::getOutput(request->arg(JSON_ID_NODE).toInt());
    if(output) {
      output->toJson(jsonResponse->getRoot(), true);
    } else {
      jsonResponse->setCode(STATUS_NOT_FOUND);
    }
  } else if(request->method() == HTTP_POST) {
    uint16_t outputID = request->arg(JSON_ID_NODE).toInt();
    uint8_t pin = request->arg(JSON_PIN_NODE).toInt();
    bool inverted = request->arg(JSON_INVERTED_NODE).equalsIgnoreCase(JSON_VALUE_TRUE);
    bool forceState = request->arg(JSON_FORCE_STATE_NODE).equalsIgnoreCase(JSON_VALUE_TRUE);
    bool defaultState = request->arg(JSON_DEFAULT_STATE_NODE).equalsIgnoreCase(JSON_VALUE_TRUE);
    uint8_t outputFlags = 0;
    if(inverted) {
      bitSet(outputFlags, OUTPUT_IFLAG_INVERT);
    }
    if(forceState) {
      bitSet(outputFlags, OUTPUT_IFLAG_RESTORE_STATE);
      if(defaultState) {
        bitSet(outputFlags, OUTPUT_IFLAG_FORCE_STATE);
      }
    }
    if(!OutputManager::createOrUpdate(outputID, pin, outputFlags)) {
      jsonResponse->setCode(STATUS_NOT_ALLOWED);
    }
  } else if(request->method() == HTTP_DELETE) {
    if(!OutputManager::remove(request->arg(JSON_ID_NODE).toInt())) {
      jsonResponse->setCode(STATUS_NOT_FOUND);
    }
  } else if(request->method() == HTTP_PUT) {
   OutputManager::toggle(request->arg(JSON_ID_NODE).toInt());
  }
  jsonResponse->setLength();
  request->send(jsonResponse);
}

void ESP32CSWebServer::handleTurnouts(AsyncWebServerRequest *request) {
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

  // if the request is GET and we do not have an ID or ADDRESS parameter we need to return an array, otherwise a single entity.
  bool needArrayResponse = (
    request->method() == HTTP_GET &&
    !request->hasArg(JSON_ID_NODE) &&
    !request->hasArg(JSON_ADDRESS_NODE));
  auto jsonResponse = new AsyncJsonResponse(needArrayResponse);
  if (request->method() == HTTP_GET && !request->hasArg(JSON_ID_NODE)) {
    bool readableStrings = true;
    if(request->hasArg(JSON_TURNOUTS_READABLE_STRINGS_NODE)) {
      readableStrings = request->arg(JSON_TURNOUTS_READABLE_STRINGS_NODE).toInt();
    }
    JsonArray array = jsonResponse->getRoot();
    TurnoutManager::getState(array, readableStrings);
  } else if (request->method() == HTTP_GET) {
    if(request->hasArg(JSON_ID_NODE)) {
      auto turnout = TurnoutManager::getTurnoutByID(request->arg(JSON_ID_NODE).toInt());
      if(turnout) {
        turnout->toJson(jsonResponse->getRoot());
      } else {
        jsonResponse->setCode(STATUS_NOT_FOUND);
      }
    } else if (request->hasArg(JSON_ADDRESS_NODE)) {
      auto turnout = TurnoutManager::getTurnoutByID(request->arg(JSON_ADDRESS_NODE).toInt());
      if(turnout) {
        turnout->toJson(jsonResponse->getRoot());
      } else {
        jsonResponse->setCode(STATUS_NOT_FOUND);
      }
    } else {
      jsonResponse->setCode(STATUS_BAD_REQUEST);
    }
  } else if(request->method() == HTTP_POST) {
    int32_t turnoutID = request->arg(JSON_ID_NODE).toInt();
    uint16_t turnoutAddress = request->arg(JSON_ADDRESS_NODE).toInt();
    int8_t turnoutSubAddress = request->arg(JSON_SUB_ADDRESS_NODE).toInt();
    TurnoutType type = (TurnoutType)request->arg(JSON_TYPE_NODE).toInt();
    // auto generate ID
    if(turnoutID == -1) {
      turnoutID = TurnoutManager::getTurnoutCount() + 1;
    }
    auto turnout = TurnoutManager::createOrUpdate((uint16_t)turnoutID, turnoutAddress, turnoutSubAddress, type);
    if(turnout) {
      turnout->toJson(jsonResponse->getRoot());
    } else {
      jsonResponse->setCode(STATUS_SERVER_ERROR);
    }
  } else if(request->method() == HTTP_DELETE) {
    if(request->hasArg(JSON_ID_NODE)) {
      TurnoutManager::removeByID(request->arg(JSON_ID_NODE).toInt());
    } else if (request->hasArg(JSON_ADDRESS_NODE)) {
      TurnoutManager::removeByAddress(request->arg(JSON_ADDRESS_NODE).toInt());
    } else {
      jsonResponse->setCode(STATUS_BAD_REQUEST);
    }
  } else if(request->method() == HTTP_PUT) {
    if(request->hasArg(JSON_ID_NODE)) {
      TurnoutManager::toggleByID(request->arg(JSON_ID_NODE).toInt());
    } else if (request->hasArg(JSON_ADDRESS_NODE)) {
      TurnoutManager::toggleByAddress(request->arg(JSON_ADDRESS_NODE).toInt());
    } else {
      jsonResponse->setCode(STATUS_BAD_REQUEST);
    }
  }
  jsonResponse->setLength();
  request->send(jsonResponse);
}

void ESP32CSWebServer::handleSensors(AsyncWebServerRequest *request) {
  auto jsonResponse = new AsyncJsonResponse(request->method() == HTTP_GET && !request->params());
  if (request->method() == HTTP_GET && !request->hasArg(JSON_ID_NODE)) {
    JsonArray array = jsonResponse->getRoot();
    SensorManager::getState(array);
  } else if (request->method() == HTTP_GET) {
    auto sensor = SensorManager::getSensor(request->arg(JSON_ID_NODE).toInt());
    if(sensor) {
      sensor->toJson(jsonResponse->getRoot());
    } else {
      jsonResponse->setCode(STATUS_NOT_FOUND);
    }
  } else if(request->method() == HTTP_POST) {
    uint16_t sensorID = request->arg(JSON_ID_NODE).toInt();
    uint8_t sensorPin = request->arg(JSON_PIN_NODE).toInt();
    bool sensorPullUp = request->arg(JSON_PULLUP_NODE).equalsIgnoreCase(JSON_VALUE_TRUE);
    if(!SensorManager::createOrUpdate(sensorID, sensorPin, sensorPullUp)) {
      jsonResponse->setCode(STATUS_NOT_ALLOWED);
    }
  } else if(request->method() == HTTP_DELETE) {
    uint16_t sensorID = request->arg(JSON_ID_NODE).toInt();
    if(SensorManager::getSensorPin(sensorID) < 0) {
      // attempt to delete S88/RemoteSensor
      jsonResponse->setCode(STATUS_NOT_ALLOWED);
    } else if(!SensorManager::remove(sensorID)) {
      jsonResponse->setCode(STATUS_NOT_FOUND);
    }
  }
  jsonResponse->setLength();
  request->send(jsonResponse);
}

void ESP32CSWebServer::handleConfig(AsyncWebServerRequest *request) {
  if (request->method() == HTTP_POST) {
    DCCPPProtocolHandler::getCommandHandler("E")->process(std::vector<std::string>());
  } else if (request->method() == HTTP_DELETE) {
    DCCPPProtocolHandler::getCommandHandler("e")->process(std::vector<std::string>());
  } else if (request->method() != HTTP_GET) {
    request->send(STATUS_BAD_REQUEST);
  }
  request->send(STATUS_OK, "application/json", configStore->getCSConfig().c_str());
}

#if S88_ENABLED
void ESP32CSWebServer::handleS88Sensors(AsyncWebServerRequest *request) {
  auto jsonResponse = new AsyncJsonResponse(true);
  if(request->method() == HTTP_GET) {
    JsonArray &array = jsonResponse->getRoot();
    S88BusManager::getState(array);
  } else if(request->method() == HTTP_POST) {
    if(!S88BusManager::createOrUpdateBus(
      request->arg(JSON_ID_NODE).toInt(),
      request->arg(JSON_PIN_NODE).toInt(),
      request->arg(JSON_COUNT_NODE).toInt())) {
      // duplicate pin/id
      jsonResponse->setCode(STATUS_NOT_ALLOWED);
    }
  } else if(request->method() == HTTP_DELETE) {
    S88BusManager::removeBus(request->arg(JSON_ID_NODE).toInt());
  }
  jsonResponse->setLength();
  request->send(jsonResponse);
}
#endif

void ESP32CSWebServer::handleRemoteSensors(AsyncWebServerRequest *request) {
  auto jsonResponse = new AsyncJsonResponse(true);
  if(request->method() == HTTP_GET) {
    JsonArray array = jsonResponse->getRoot();
    RemoteSensorManager::getState(array);
  } else if(request->method() == HTTP_POST) {
    RemoteSensorManager::createOrUpdate(request->arg(JSON_ID_NODE).toInt(),
      request->arg(JSON_VALUE_NODE).toInt());
  } else if(request->method() == HTTP_DELETE) {
    RemoteSensorManager::remove(request->arg(JSON_ID_NODE).toInt());
  }
  jsonResponse->setLength();
  request->send(jsonResponse);
}

void ESP32CSWebServer::handleLocomotive(AsyncWebServerRequest *request) {
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
  const String url = request->url();
  auto jsonResponse = new AsyncJsonResponse(request->method() == HTTP_GET && !request->params());
  jsonResponse->setCode(STATUS_OK);
  // check if we have an eStop command, we don't care how this gets sent to the
  // command station (method) so check it first
  if(url.endsWith("/estop")) {
    LocomotiveManager::emergencyStop();
  } else if(url.indexOf("/roster") > 0) {
    if(request->method() == HTTP_GET && !request->hasArg(JSON_ADDRESS_NODE)) {
      LocomotiveManager::getRosterEntries(jsonResponse->getRoot());
    } else if (request->hasArg(JSON_ADDRESS_NODE)) {
      if(request->method() == HTTP_DELETE) {
        LocomotiveManager::removeRosterEntry(request->arg(JSON_ADDRESS_NODE).toInt());
      } else {
        RosterEntry *entry = LocomotiveManager::getRosterEntry(request->arg(JSON_ADDRESS_NODE).toInt());
        if(request->method() == HTTP_PUT || request->method() == HTTP_POST) {
          if(request->hasArg(JSON_DESCRIPTION_NODE)) {
            entry->setDescription(request->arg(JSON_DESCRIPTION_NODE).c_str());
          }
          if(request->hasArg(JSON_TYPE_NODE)) {
            entry->setType(request->arg(JSON_TYPE_NODE).c_str());
          }
          if(request->hasArg(JSON_IDLE_ON_STARTUP_NODE)) {
            entry->setIdleOnStartup(request->arg(JSON_IDLE_ON_STARTUP_NODE).equalsIgnoreCase(JSON_VALUE_TRUE));
          }
          if(request->hasArg(JSON_DEFAULT_ON_THROTTLE_NODE)) {
            entry->setDefaultOnThrottles(request->arg(JSON_DEFAULT_ON_THROTTLE_NODE).equalsIgnoreCase(JSON_VALUE_TRUE));
          }
        }
        entry->toJson(jsonResponse->getRoot());
      }
    }
  } else {
    // Since it is not an eStop or roster command we need to check the request
    // method and ensure it contains the required arguments otherwise the
    // request should be rejected
    if(request->method() == HTTP_GET && !request->hasArg(JSON_ADDRESS_NODE)) {
      // get all active locomotives
      LocomotiveManager::getActiveLocos(jsonResponse->getRoot()); 
    } else if (request->hasArg(JSON_ADDRESS_NODE)) {
      auto loco = LocomotiveManager::getLocomotive(request->arg(JSON_ADDRESS_NODE).toInt());
      if(request->method() == HTTP_PUT || request->method() == HTTP_POST) {
        // Creation / Update of active locomotive
        bool needUpdate = false;
        if(request->hasArg(JSON_IDLE_NODE) && request->arg(JSON_IDLE_NODE).equalsIgnoreCase(JSON_VALUE_TRUE)) {
          loco->setIdle();
          needUpdate = true;
        }
        if(request->hasArg(JSON_DIRECTION_NODE)) {
          loco->setDirection(request->arg(JSON_DIRECTION_NODE).equalsIgnoreCase(JSON_VALUE_FORWARD));
          needUpdate = true;
        }
        if(request->hasArg(JSON_SPEED_NODE)) {
          loco->setSpeed(request->arg(JSON_SPEED_NODE).toInt());
          needUpdate = true;
        }
        for(uint8_t funcID = 0; funcID <=28 ; funcID++) {
          String fArg = "f" + String(funcID);
          if(request->hasArg(fArg.c_str())) {
            loco->setFunction(funcID, request->arg(fArg.c_str()).equalsIgnoreCase(JSON_VALUE_TRUE));
          }
        }
        if(needUpdate) {
          loco->sendLocoUpdate(true);
          loco->showStatus();
        }
      } else if(request->method() == HTTP_DELETE) {
        // Removal of an active locomotive
        LocomotiveManager::removeLocomotive(request->arg(JSON_ADDRESS_NODE).toInt());
#if NEXTION_ENABLED
        static_cast<NextionThrottlePage *>(nextionPages[THROTTLE_PAGE])->invalidateLocomotive(request->arg(JSON_ADDRESS_NODE).toInt());
#endif
      }
      loco->toJson(jsonResponse->getRoot());
    } else {
      // missing arg or unknown request
      jsonResponse->setCode(STATUS_BAD_REQUEST);
    }
  }
  jsonResponse->setLength();
  request->send(jsonResponse);
}

void ESP32CSWebServer::handleOTA(AsyncWebServerRequest *request) {
  request->send(STATUS_OK, "text/plain", "OTA Upload Complete");
}

void ESP32CSWebServer::handleFeatures(AsyncWebServerRequest *request) {
  auto jsonResponse = new AsyncJsonResponse();
  JsonObject root = jsonResponse->getRoot();
#if S88_ENABLED
  root[JSON_S88_NODE] = JSON_VALUE_TRUE;
  root[JSON_S88_SENSOR_BASE_NODE] = S88_FIRST_SENSOR;
#else
  root[JSON_S88_NODE] = JSON_VALUE_FALSE;
#endif
  jsonResponse->setCode(STATUS_OK);
  jsonResponse->setLength();
  request->send(jsonResponse);
}

#define STREAM_RESOURCE_NO_REDIRECT(request, uri, mimeType, totalSize, resource) \
  if (request->url() == uri) { \
    LOG(INFO, "[WebSrv %s] Sending %s from MEMORY (%d, %s)", ipv4_to_string(request->client()->getRemoteAddress()).c_str(), uri, totalSize, mimeType); \
    response = request->beginResponse_P(STATUS_OK, mimeType, resource, totalSize); \
    if(String(mimeType).startsWith("text/")) { \
      response->addHeader("Content-Encoding", "gzip"); \
    } \
  }

#if WIFI_ENABLE_SOFT_AP
#define STREAM_RESOURCE(request, uri, fallback, mimeType, totalSize, resource) \
  if (request->url() == uri && softAPAddress_.compare(request->host().c_str()) == 0) { \
    LOG(INFO, "[WebSrv %s] Sending %s from MEMORY (%d, %s)", ipv4_to_string(request->client()->getRemoteAddress()).c_str(), uri, totalSize, mimeType); \
    response = request->beginResponse_P(STATUS_OK, mimeType, resource, totalSize); \
    if(String(mimeType).startsWith("text/")) { \
      response->addHeader("Content-Encoding", "gzip"); \
    } \
  } else if (request->url() == uri) { \
    LOG(INFO, "[WebSrv %s] Requested %s => CDN %s", ipv4_to_string(request->client()->getRemoteAddress()).c_str(), uri, fallback); \
    request->redirect(fallback); \
  }
#else
#define STREAM_RESOURCE(request, uri, fallback, mimeType, totalSize, resource) \
  if (request->url() == uri) { \
    LOG(INFO, "[WebSrv %s] Requested %s => CDN %s", ipv4_to_string(request->client()->getRemoteAddress()).c_str(), uri, fallback); \
    request->redirect(fallback); \
  }
#endif

void ESP32CSWebServer::streamResource(AsyncWebServerRequest *request) {
  const char * htmlBuildTime = __DATE__ " " __TIME__;
  if (request->header("If-Modified-Since").equals(htmlBuildTime)) {
    request->send(STATUS_NOT_MODIFIED);
  } else {
    AsyncWebServerResponse *response = nullptr;
    STREAM_RESOURCE_NO_REDIRECT(request, "/index.html", "text/html", indexHtmlGz_size, indexHtmlGz)
    STREAM_RESOURCE(request, "/jquery.min.js", "https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js", "text/javascript", jqueryJsGz_size, jqueryJsGz)
    STREAM_RESOURCE(request, "/jquery.mobile-1.5.0-rc1.min.js", "https://code.jquery.com/mobile/1.5.0-rc1/jquery.mobile-1.5.0-rc1.min.js", "text/javascript", jqueryMobileJsGz_size, jqueryMobileJsGz)
    STREAM_RESOURCE(request, "/jquery.mobile-1.5.0-rc1.min.css", "https://code.jquery.com/mobile/1.5.0-rc1/jquery.mobile-1.5.0-rc1.min.css", "text/css", jqueryMobileCssGz_size, jqueryMobileCssGz)
    STREAM_RESOURCE(request, "/jquery.simple.websocket.min.js", "https://cdn.rawgit.com/jbloemendal/jquery-simple-websocket/master/dist/jquery.simple.websocket.min.js", "text/javascript", jquerySimpleWebSocketGz_size, jquerySimpleWebSocketGz)
    STREAM_RESOURCE(request, "/jqClock-lite.min.js", "https://cdn.rawgit.com/JohnRDOrazio/jQuery-Clock-Plugin/master/jqClock-lite.min.js", "text/javascript", jqClockGz_size, jqClockGz)
    STREAM_RESOURCE(request, "/images/ajax-loader.gif", "https://code.jquery.com/mobile/1.5.0-rc1/images/ajax-loader.gif", "image/gif", ajaxLoader_size, ajaxLoader)
    if(response) {
      response->addHeader("Last-Modified", htmlBuildTime);
      request->send(response);
    } else {
      request->send(STATUS_NOT_FOUND, "text/plain", "URI Not Found");
    }
  }
}

// Known captive portal checks to respond with http 200 and redirect link
String portalCheckRedirect[] = {
  "/generate_204",                  // Android
  "/gen_204",                       // Android 9.0
  "/mobile/status.php",             // Android 8.0 (Samsung s9+)
  "/ncsi.txt",                      // Windows
  "/success.txt",                   // OSX
  "/hotspot-detect.html",           // iOS 8/9
  "/hotspotdetect.html",            // iOS 8/9
  "/library/test/success.html"      // iOS 8/9
  "/kindle-wifi/wifiredirect.html"  // Kindle
  "/kindle-wifi/wifistub.html"      // Kindle
};

static constexpr const char * const REDIRECT_HTML =
  "<html>"
  "<title>ESP32 Command Station v%s</title>"                                          // VERSION
  "<meta http-equiv=\"refresh\" content=\"5;url='http://%s/index.html'\" /> "         // softAPAddress_
  "<body>"
  "<h1>Welcome to the ESP32 Command Station</h1>"
  "<p>Click <a href=\"http://%s/index.html\">here</a> for ESP32 Command Station if "  // softAPAddress_
  "you are not automatically redirected in a few seconds.</p>"
  "</body>"
  "</html>";

void ESP32CSWebServer::notFoundHandler(AsyncWebServerRequest *request) {
#if WIFI_ENABLE_SOFT_AP
  for(auto uri : portalCheckRedirect) {
    if(request->url() == uri) {
      LOG(INFO,
          "[WebSrv %s] Requested %s%s: Appears to be a captive portal check, redirecting to http://%s/index.html",
          request->client()->remoteIP().toString().c_str(),
          request->host().c_str(),
          request->url().c_str(),
          softAPAddress_.c_str());
      request->send(STATUS_OK,
                    "text/html",
                    StringPrintf(REDIRECT_HTML,
                                 VERSION,
                                 softAPAddress_.c_str(),
                                 softAPAddress_.c_str()).c_str());
      return;
    }
  }
#endif
  LOG(INFO,
      "[WebSrv %s] 404: %s%s",
      ipv4_to_string(request->client()->getRemoteAddress()).c_str(),
      request->host().c_str(),
      request->url().c_str());
  request->send(STATUS_NOT_FOUND, "", "");
}