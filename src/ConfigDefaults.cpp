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

#include <utils/constants.hxx>

// GCC pre-compiler trick to expand the value from a #define constant
#define DEFAULT_CONST_EXPAND_VALUE(var, value) DEFAULT_CONST(var, value)

#ifndef ESP32CS_EXTERNAL_CONFIGURATION
#include "Config.h"
#endif

#include "DefaultConfigs.h"

///////////////////////////////////////////////////////////////////////////////
// This flag will clear the stored configuration data causing the command
// station to regenerate the configuration from scratch. This is usually not
// necessary.
///////////////////////////////////////////////////////////////////////////////
DEFAULT_CONST_FALSE(cs_force_factory_reset);

///////////////////////////////////////////////////////////////////////////////
// This flag will force a factory reset by removing the LCC_CDI_FILE and
// LCC_CONFIG_FILE before starting the OpenMRN stack. This should not normally
// be required.
///////////////////////////////////////////////////////////////////////////////
DEFAULT_CONST_EXPAND_VALUE(lcc_force_factory_reset, LCC_FORCE_FACTORY_RESET_ON_STARTUP);

///////////////////////////////////////////////////////////////////////////////
// This flag controls the fsync call inteval for the LCC node config file when
// using the SD card as the storage device. When using SPIFFS as the storage
// device this setting will not be used.
///////////////////////////////////////////////////////////////////////////////
DEFAULT_CONST(lcc_sd_sync_interval_sec, 10);

///////////////////////////////////////////////////////////////////////////////
// This flag controls the printing of all LCC GridConnect packets.
///////////////////////////////////////////////////////////////////////////////
DEFAULT_CONST_FALSE(lcc_print_all_packets);

///////////////////////////////////////////////////////////////////////////////
// This flag controls automatic creation of Locomotive roster entries based on
// the request from the LCC FindProtocolServer -> AllTrainNodes::allocate_node
// call.
///////////////////////////////////////////////////////////////////////////////
DEFAULT_CONST_TRUE(cs_train_db_auto_create_entries);

///////////////////////////////////////////////////////////////////////////////
// This flag controls the automatic persistence of the locomotive roster list.
///////////////////////////////////////////////////////////////////////////////
DEFAULT_CONST(cs_train_db_auto_persist_sec, 30);

///////////////////////////////////////////////////////////////////////////////
// This flag will print a list of FreeRTOS tasks every ~5min. This is not
// recommended to be enabled except during debugging sessions as it will cause
// the FreeRTOS scheduler to remain in a "locked" state for an extended period.
///////////////////////////////////////////////////////////////////////////////
DEFAULT_CONST_FALSE(cs_task_list_report);
DEFAULT_CONST(cs_task_list_list_interval_sec, 300);

///////////////////////////////////////////////////////////////////////////////
// This flag controls how often the CS task stats will be reported.
///////////////////////////////////////////////////////////////////////////////
DEFAULT_CONST(cs_task_stats_report_interval_sec, 45);

///////////////////////////////////////////////////////////////////////////////
// This is the number of pending dcc::Packet objects that the LocalTrackIf will
// use in it's FixedPool.
///////////////////////////////////////////////////////////////////////////////
DEFAULT_CONST(cs_track_pool_size, 5);

///////////////////////////////////////////////////////////////////////////////
// Enabling this will print all RailCom packet data as it arrives at the hub.
///////////////////////////////////////////////////////////////////////////////
DEFAULT_CONST_FALSE(enable_railcom_packet_dump);

///////////////////////////////////////////////////////////////////////////////
// This controls the ability to enable RailCom via configuration parameters.
///////////////////////////////////////////////////////////////////////////////
DEFAULT_CONST_FALSE(cs_railcom_enabled);

///////////////////////////////////////////////////////////////////////////////
// This is the number of pending dcc::Packet objects that the RMT driver will
// allow to be queued for outbound delivery.
///////////////////////////////////////////////////////////////////////////////
DEFAULT_CONST(rmt_packet_queue_ops, 10);
DEFAULT_CONST(rmt_packet_queue_prog, 5);

///////////////////////////////////////////////////////////////////////////////
// HC12 configuration settings
///////////////////////////////////////////////////////////////////////////////
DEFAULT_CONST(cs_hc12_buffer_size, 256);
DEFAULT_CONST(cs_hc12_uart_speed, 19200);

///////////////////////////////////////////////////////////////////////////////
// This controls how many DCC e-stop packets will be generated before the
// e-stop handler will discontinue sending packets.
///////////////////////////////////////////////////////////////////////////////
DEFAULT_CONST(cs_estop_packet_count, 200);

///////////////////////////////////////////////////////////////////////////////
// Status LED configuration settings
///////////////////////////////////////////////////////////////////////////////
DEFAULT_CONST_EXPAND_VALUE(status_led_enabled, STATUS_LED_ENABLED);
DEFAULT_CONST_EXPAND_VALUE(status_led_pin, STATUS_LED_DATA_PIN);
DEFAULT_CONST_EXPAND_VALUE(status_led_brightness, STATUS_LED_BRIGHTNESS);

DEFAULT_CONST(status_led_update_interval_msec, 450);

///////////////////////////////////////////////////////////////////////////////
// Httpd constants
///////////////////////////////////////////////////////////////////////////////
DEFAULT_CONST(httpd_server_stack_size, 4096);
DEFAULT_CONST(httpd_server_priority, 0);
DEFAULT_CONST(httpd_header_chunk_size, 256);
DEFAULT_CONST(httpd_body_chunk_size, 3072);
DEFAULT_CONST(httpd_response_chunk_size, 2048);
DEFAULT_CONST(httpd_max_req_size, 4194304);
DEFAULT_CONST(httpd_max_req_per_connection, 2);
DEFAULT_CONST(httpd_req_timeout_ms, 5);
DEFAULT_CONST(httpd_socket_receive_timeout_ms, 100);
DEFAULT_CONST(httpd_websocket_timeout_ms, 500);
DEFAULT_CONST(httpd_websocket_max_frame_size, 256);
DEFAULT_CONST(httpd_websocket_max_read_attempts, 5);
DEFAULT_CONST(httpd_websocket_max_write_attempts, 5);