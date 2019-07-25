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


#include "cdi/CSConfigDescriptor.h"
#include "stateflows/MonitoredHBridge.h"

#include <openlcb/Node.hxx>
#include <openlcb/SimpleStack.hxx>

void setup_hbridge_event_handlers(openlcb::Node *);
void register_monitored_hbridge(openlcb::SimpleCanStack *
                              , const adc1_channel_t
                              , const gpio_num_t
                              , const gpio_num_t
                              , const uint32_t
                              , const uint32_t
                              , const std::string &
                              , const std::string &
                              , const TrackOutputConfig &
                              , const bool=false);

void get_hbridge_status_json(JsonArray);
bool is_track_power_on();
void enable_all_hbridges();
void enable_named_hbridge(std::string);
void disable_all_hbridges();
void disable_named_hbridge(std::string);
uint32_t get_hbridge_sample(std::string);
uint32_t get_hbridge_max_amps(std::string);
void broadcast_all_hbridge_statuses();
void broadcast_named_hbridge_status(std::string);
std::string get_hbridge_info_screen_data();
