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

#ifndef _DEFAULT_CONFIGS_H_
#define _DEFAULT_CONFIGS_H_

///////////////////////////////////////////////////////////////////////////////
// Command Station Feature configuration
///////////////////////////////////////////////////////////////////////////////

#ifndef ALLOW_USAGE_OF_RESTRICTED_GPIO_PINS
#define ALLOW_USAGE_OF_RESTRICTED_GPIO_PINS false
#endif

#ifndef ENABLE_OUTPUTS
#define ENABLE_OUTPUTS true
#endif

#ifndef ENABLE_SENSORS
#define ENABLE_SENSORS true
#endif

#ifndef ENABLE_TASK_MONITOR
#define ENABLE_TASK_MONITOR false
#endif

#ifndef CPULOAD_REPORTING
#define CPULOAD_REPORTING false
#endif

#ifndef NEXTION_ENABLED
#define NEXTION_ENABLED false
#endif

#ifndef HC12_RADIO_ENABLED
#define HC12_RADIO_ENABLED false
#endif

///////////////////////////////////////////////////////////////////////////////
// H-Bridge default configuration
///////////////////////////////////////////////////////////////////////////////

#ifndef OPS_TRACK_PREAMBLE_BITS
#define OPS_TRACK_PREAMBLE_BITS 16
#endif

#ifndef PROG_TRACK_PREAMBLE_BITS
#define PROG_TRACK_PREAMBLE_BITS 22
#endif

#ifndef ENERGIZE_OPS_TRACK_ON_STARTUP
#define ENERGIZE_OPS_TRACK_ON_STARTUP false
#endif

///////////////////////////////////////////////////////////////////////////////
// WiFi default configuration
///////////////////////////////////////////////////////////////////////////////

#ifndef HOSTNAME_PREFIX
#define HOSTNAME_PREFIX "esp32cs_"
#endif

#ifndef WIFI_ENABLE_SOFT_AP
#define WIFI_ENABLE_SOFT_AP false
#endif

#ifndef WIFI_SOFT_AP_CHANNEL
#define WIFI_SOFT_AP_CHANNEL 6
#endif

///////////////////////////////////////////////////////////////////////////////
// LCC default configuration
///////////////////////////////////////////////////////////////////////////////

// This defines where on the filesystem LCC configuration data will be
// persisted.
#define LCC_CONFIG_DIR "/LCC"

// This is where the CDI will be persisted on the filesystem and sent from
// on-demand when the NODE CDI information is requested.
#define LCC_CDI_FILE "/LCC/cdi.xml"

// This is where the NODE persistent configuration will be stored on the
// filesystem.
#define LCC_CONFIG_FILE "/LCC/config"

// This is the default node ID assigned to the CS.
#ifndef LCC_NODE_ID
#define LCC_NODE_ID 0x050101013F00
#endif

///////////////////////////////////////////////////////////////////////////////
// InfoScreen default configuration
///////////////////////////////////////////////////////////////////////////////

#if (defined(INFO_SCREEN_LCD) && INFO_SCREEN_LCD) || (defined(INFO_SCREEN_OLED) && INFO_SCREEN_OLED)
#define INFO_SCREEN_ENABLED true
#if (defined(INFO_SCREEN_LCD) && INFO_SCREEN_LCD)
#define INFO_SCREEN_OLED false
#endif
#if (defined(INFO_SCREEN_OLED) && INFO_SCREEN_OLED)
#define INFO_SCREEN_LCD false
#endif
#else
#define INFO_SCREEN_ENABLED false
#define INFO_SCREEN_LCD false
#define INFO_SCREEN_OLED false
#endif

#ifndef INFO_SCREEN_SDA_PIN
#define INFO_SCREEN_SDA_PIN SDA
#endif

#ifndef INFO_SCREEN_SCL_PIN
#define INFO_SCREEN_SCL_PIN SCL
#endif

#ifndef INFO_SCREEN_OLED_LINES
#define INFO_SCREEN_OLED_LINES 5
#endif

///////////////////////////////////////////////////////////////////////////////
// LocoNet default configuration
///////////////////////////////////////////////////////////////////////////////

#ifndef LOCONET_ENABLED
#define LOCONET_ENABLED false
#endif

#ifndef LOCONET_INVERTED_LOGIC
#define LOCONET_INVERTED_LOGIC false
#endif

#ifndef LOCONET_ENABLE_RX_PIN_PULLUP
#define LOCONET_ENABLE_RX_PIN_PULLUP false
#endif

///////////////////////////////////////////////////////////////////////////////
// Status LED default configuration
///////////////////////////////////////////////////////////////////////////////

#ifndef STATUS_LED_ENABLED
#define STATUS_LED_TYPE WS281X_800K
#define STATUS_LED_COLOR_ORDER RGB
#define STATUS_LED_ENABLED false
#endif

#ifndef STATUS_LED_COLOR_ORDER
#define STATUS_LED_COLOR_ORDER RGB
#endif

#ifndef STATUS_LED_TYPE
#define STATUS_LED_TYPE WS281X
#endif

#ifndef STATUS_LED_BRIGHTNESS
#define STATUS_LED_BRIGHTNESS 128
#endif

///////////////////////////////////////////////////////////////////////////////
// S88 default configuration
///////////////////////////////////////////////////////////////////////////////
constexpr uint16_t S88_MAX_SENSORS_PER_BUS = 512;

#ifndef S88_ENABLED
#define S88_ENABLED false
#endif

#ifndef S88_FIRST_SENSOR
#define S88_FIRST_SENSOR S88_MAX_SENSORS_PER_BUS
#endif

/////////////////////////////////////////////////////////////////////////////////////
// ESP32 CS Internal flags default configuration
/////////////////////////////////////////////////////////////////////////////////////

#ifndef ENABLE_TASK_LIST_REPORTING
#define ENABLE_TASK_LIST_REPORTING false
#endif

#ifndef CPULOAD_REPORTING
#define CPULOAD_REPORTING false
#endif

#endif // _DEFAULT_CONFIGS_H_