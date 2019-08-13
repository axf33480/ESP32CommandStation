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
// This flag will print a list of FreeRTOS tasks every ~5min. This is not
// recommended to be enabled except during debugging sessions as it will cause
// the FreeRTOS scheduler to remain in a "locked" state for an extended period.
///////////////////////////////////////////////////////////////////////////////
DEFAULT_CONST_FALSE(cs_task_list_reporting);

///////////////////////////////////////////////////////////////////////////////
// This flag will cause cpu utilization metrics to be collected and reported by
// the LCC CpuLoad and CpuLoadLog system.
///////////////////////////////////////////////////////////////////////////////
DEFAULT_CONST_FALSE(cs_task_list_report);

DEFAULT_CONST(cs_task_list_report_interval_sec, 45);

DEFAULT_CONST(cs_task_list_stats_interval_sec, 300);

///////////////////////////////////////////////////////////////////////////////
// This flag will force a factory reset by removing the LCC_CDI_FILE and
// LCC_CONFIG_FILE before starting the OpenMRN stack. This should not normally
// be required.
///////////////////////////////////////////////////////////////////////////////
#if !LCC_FORCE_FACTORY_RESET_ON_STARTUP
DEFAULT_CONST_FALSE(lcc_force_factory_reset);
#else
DEFAULT_CONST_TRUE(lcc_force_factory_reset);
#endif

///////////////////////////////////////////////////////////////////////////////
// Enabling this will print all RailCom packet data as it arrives at the hub.
///////////////////////////////////////////////////////////////////////////////
DEFAULT_CONST_FALSE(enable_railcom_packet_dump);

///////////////////////////////////////////////////////////////////////////////
// This is the number of pending dcc::Packet objects that the RMT driver will
// allow to be queued for outbound delivery.
///////////////////////////////////////////////////////////////////////////////
DEFAULT_CONST(rmt_packet_queue_ops, 10);
DEFAULT_CONST(rmt_packet_queue_prog, 5);

///////////////////////////////////////////////////////////////////////////////
// HC12 configuration settings
///////////////////////////////////////////////////////////////////////////////
#if !HC12_RADIO_ENABLED
DEFAULT_CONST_FALSE(cs_hc12_enabled);
#else
DEFAULT_CONST_TRUE(cs_hc12_enabled);
#endif
DEFAULT_CONST(cs_hc12_buffer_size, 256);
DEFAULT_CONST(cs_hc12_uart_num, HC12_UART_NUM);
DEFAULT_CONST(cs_hc12_uart_speed, 19200);
DEFAULT_CONST(cs_hc12_rx_pin, HC12_RX_PIN);
DEFAULT_CONST(cs_hc12_rx_pin, HC12_TX_PIN);