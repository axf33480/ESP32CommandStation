/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2019 Mike Dunston

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

#ifndef JSON_CONSTS_H_
#define JSON_CONSTS_H_

constexpr const char * JSON_FILE_NODE = "file";

constexpr const char * JSON_NAME_NODE = "name";
constexpr const char * JSON_STATE_NODE = "state";
constexpr const char * JSON_USAGE_NODE = "usage";

constexpr const char * JSON_COUNT_NODE = "count";

constexpr const char * JSON_ADDRESS_NODE = "address";

constexpr const char * JSON_SUB_ADDRESS_NODE = "subAddress";
constexpr const char * JSON_BOARD_ADDRESS_NODE = "boardAddress";
constexpr const char * JSON_SPEED_NODE = "speed";
constexpr const char * JSON_DIRECTION_NODE = "dir";
constexpr const char * JSON_ORIENTATION_NODE = "orientation";
constexpr const char * JSON_DESCRIPTION_NODE = "description";
constexpr const char * JSON_TYPE_NODE = "type";
constexpr const char * JSON_IDLE_NODE = "idle";

constexpr const char * JSON_IDLE_ON_STARTUP_NODE = "idleOnStartup";
constexpr const char * JSON_DEFAULT_ON_THROTTLE_NODE = "defaultOnThrottles";

constexpr const char * JSON_FUNCTIONS_NODE = "functions";
constexpr const char * JSON_LOCOS_NODE = "locos";
constexpr const char * JSON_LOCO_NODE = "loco";

constexpr const char * JSON_CONSIST_NODE = "consist";
constexpr const char * JSON_CONSISTS_NODE = "consists";
constexpr const char * JSON_DECODER_ASSISTED_NODE = "decoderAssisted";

constexpr const char * JSON_OUTPUTS_NODE = "outputs";
constexpr const char * JSON_ID_NODE = "id";
constexpr const char * JSON_PIN_NODE = "pin";
constexpr const char * JSON_FLAGS_NODE = "flags";
constexpr const char * JSON_INVERTED_NODE = "inverted";
constexpr const char * JSON_FORCE_STATE_NODE = "forceState";
constexpr const char * JSON_DEFAULT_STATE_NODE = "defaultState";

constexpr const char * JSON_SENSORS_NODE = "sensors";
constexpr const char * JSON_PULLUP_NODE = "pullUp";

constexpr const char * JSON_TURNOUTS_NODE = "turnouts";
constexpr const char * JSON_TURNOUTS_READABLE_STRINGS_NODE = "readableStrings";

constexpr const char * JSON_S88_NODE = "s88";
constexpr const char * JSON_S88_SENSOR_BASE_NODE = "sensorIDBase";

constexpr const char * JSON_PROG_ON_MAIN = "pom";
constexpr const char * JSON_CV_NODE = "cv";
constexpr const char * JSON_VALUE_NODE = "value";
constexpr const char * JSON_CV_BIT_NODE = "bit";
constexpr const char * JSON_IDENTIFY_NODE = "identify";
constexpr const char * JSON_ADDRESS_MODE_NODE = "addressMode";
constexpr const char * JSON_SPEED_TABLE_NODE = "speedTable";
constexpr const char * JSON_DECODER_VERSION_NODE = "version";
constexpr const char * JSON_DECODER_MANUFACTURER_NODE = "manufacturer";
constexpr const char * JSON_CREATE_NODE = "create";
constexpr const char * JSON_OVERALL_STATE_NODE = "overallState";
constexpr const char * JSON_LAST_UPDATE_NODE = "lastUpdate";

constexpr const char * JSON_WIFI_NODE = "wifi";
constexpr const char * JSON_WIFI_MODE_NODE = "mode";
constexpr const char * JSON_WIFI_SSID_NODE = "ssid";
constexpr const char * JSON_WIFI_PASSWORD_NODE = "password";
constexpr const char * JSON_WIFI_STATION_NODE = "station";
constexpr const char * JSON_WIFI_STATION_IP_NODE = "ip";
constexpr const char * JSON_WIFI_STATION_GATEWAY_NODE = "gateway";
constexpr const char * JSON_WIFI_STATION_NETMASK_NODE = "netmask";
constexpr const char * JSON_WIFI_DNS_NODE = "dns";

constexpr const char * JSON_VALUE_STATION_IP_MODE_STATIC = "static";
constexpr const char * JSON_VALUE_STATION_IP_MODE_DHCP = "dhcp";

constexpr const char * JSON_VALUE_WIFI_MODE_SOFTAP_ONLY = "softap";
constexpr const char * JSON_VALUE_WIFI_MODE_SOFTAP_STATION = "softap-station";
constexpr const char * JSON_VALUE_WIFI_MODE_STATION_ONLY = "station";

constexpr const char * JSON_VALUE_FORWARD = "FWD";
constexpr const char * JSON_VALUE_REVERSE = "REV";
constexpr const char * JSON_VALUE_TRUE = "true";
constexpr const char * JSON_VALUE_FALSE = "false";
constexpr const char * JSON_VALUE_NORMAL = "Normal";
constexpr const char * JSON_VALUE_OFF = "Off";
constexpr const char * JSON_VALUE_ON = "On";
constexpr const char * JSON_VALUE_FAULT = "Fault";
constexpr const char * JSON_VALUE_ERROR = "Error";
constexpr const char * JSON_VALUE_THROWN = "Thrown";
constexpr const char * JSON_VALUE_CLOSED = "Closed";
constexpr const char * JSON_VALUE_LONG_ADDRESS = "Long Address";
constexpr const char * JSON_VALUE_SHORT_ADDRESS = "Short Address";
constexpr const char * JSON_VALUE_MOBILE_DECODER = "Mobile Decoder";
constexpr const char * JSON_VALUE_STATIONARY_DECODER = "Stationary Decoder";

constexpr const char * ESP32_CS_CONFIG_JSON = "esp32cs-config.json";

#endif // JSON_CONSTS_H_