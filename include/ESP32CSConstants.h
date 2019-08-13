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

#ifndef _COMMAND_STATION_CONFIG_H_
#define _COMMAND_STATION_CONFIG_H_

#include <utils/constants.hxx>

DECLARE_CONST(cs_force_factory_reset);
DECLARE_CONST(cs_task_list_report);
DECLARE_CONST(cs_task_list_report_interval_sec);
DECLARE_CONST(cs_task_list_stats_interval_sec);

DECLARE_CONST(cs_cpu_reporting);

DECLARE_CONST(lcc_force_factory_reset);

DECLARE_CONST(enable_railcom_packet_dump);

DECLARE_CONST(rmt_packet_queue_ops);
DECLARE_CONST(rmt_packet_queue_prog);

DECLARE_CONST(cs_hc12_enabled);
DECLARE_CONST(cs_hc12_uart_num);
DECLARE_CONST(cs_hc12_uart_speed);
DECLARE_CONST(cs_hc12_buffer_size);
DECLARE_CONST(cs_hc12_rx_pin);
DECLARE_CONST(cs_hc12_tx_pin);

#endif