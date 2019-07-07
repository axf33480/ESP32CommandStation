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

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE WiFi Parameters
//
#define SSID_NAME "SSID_NAME_HERE"
#define SSID_PASSWORD "SSID_PASSWORD_HERE"

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE STATIC IP ADDRESS DETAILS OR LEAVE COMMENTED FOR DHCP
//

//#define WIFI_STATIC_IP_ADDRESS "10.0.0.155"
//#define WIFI_STATIC_IP_GATEWAY "10.0.0.1"
//#define WIFI_STATIC_IP_SUBNET "255.255.255.0"

// WIFI_STATIC_IP_DNS is optional, if not defined the value below will be used
// automatically. This is a Google provided DNS server.
//#define WIFI_STATIC_IP_DNS "8.8.8.8"

/////////////////////////////////////////////////////////////////////////////////////
//
// Remote Sensors are enabled by default, the following parameters can be used
// to control their behavior.

//#define SCAN_REMOTE_SENSORS_ON_STARTUP true
//#define REMOTE_SENSORS_PREFIX "sensor"
//#define REMOTE_SENSORS_DECAY 60000
//#define REMOTE_SENSORS_FIRST_SENSOR 100

/////////////////////////////////////////////////////////////////////////////////////
//
// This defines the hostname prefix to use for the ESP32 Command Station. The LCC
// Node ID will be appended to this value.
//
#define HOSTNAME_PREFIX "ESP32CS_"

//#define WIFI_ENABLE_SOFT_AP true
//#define WIFI_SOFT_AP_CHANNEL 6
//#define WIFI_SOFT_AP_MAX_CLIENTS 4

/////////////////////////////////////////////////////////////////////////////////////
