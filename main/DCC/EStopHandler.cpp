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

#include "ESP32CommandStation.h"

namespace esp32cs
{

void EStopHandler::set_state(bool new_value)
{
  if (new_value)
  {
    // TODO: add helper method on AllTrainNodes for this.
    auto  trains = Singleton<AllTrainNodes>::instance();
    for (size_t id = 0; id < trains->size(); id++)
    {
      auto node = trains->get_train_node_id(id);
      if (node)
      {
        trains->get_train_impl(node)->set_emergencystop();
      }
    }
    {
      OSMutexLock l(&lock_);
      remaining_ = CONFIG_DCC_ESTOP_PACKET_COUNT;
      packet_processor_add_refresh_source(this, UpdateLoopBase::ESTOP_PRIORITY);
    }
  }
  else
  {
    OSMutexLock l(&lock_);
    remaining_ = 0;
    packet_processor_remove_refresh_source(this);
  }
}

void EStopHandler::get_next_packet(unsigned code, Packet* packet)
{
  packet->set_dcc_speed14(DccShortAddress(0), true, false
                        , Packet::EMERGENCY_STOP);
  {
    OSMutexLock l(&lock_);
    remaining_--;
    if (remaining_ <= 0)
    {
      packet_processor_remove_refresh_source(this);
    }
  }
}

}
