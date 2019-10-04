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

#ifndef ESTOP_HANDLER_H_
#define ESTOP_HANDLER_H_

#include <memory>

#include <dcc/PacketSource.hxx>
#include <dcc/SimpleUpdateLoop.hxx>
#include <openlcb/Defs.hxx>
#include <openlcb/EventHandlerTemplates.hxx>
#include <os/OS.hxx>
#include <utils/Singleton.hxx>
#include "AllTrainNodes.hxx"

#include "ESP32CSConstants.h"

extern std::unique_ptr<commandstation::AllTrainNodes> trainNodes;

namespace esp32cs
{

using openlcb::BitEventInterface;
using openlcb::Defs;
using openlcb::EventState;
using openlcb::Node;
using dcc::DccShortAddress;
using dcc::Packet;
using dcc::NonTrainPacketSource;
using dcc::SimpleUpdateLoop;
using dcc::UpdateLoopBase;

// Event handler for the E-Stop well known events. This will generate a
// continuous stream of e-stop DCC packets until the E-Stop event has been
// received or the state has been reset via API.
class EStopHandler
  : public openlcb::BitEventInterface
  , public Singleton<EStopHandler>
  , public NonTrainPacketSource
{
public:
  EStopHandler(Node *node)
    : BitEventInterface(Defs::CLEAR_EMERGENCY_STOP_EVENT
                      , Defs::EMERGENCY_STOP_EVENT)
    , node_(node)
    , remaining_(0)
  {
  }

  EventState get_current_state() override
  {
    return EventState::INVALID;
  }

  void set_state(bool new_value) override
  {
    if (new_value)
    {
      for (size_t id = 0; id < trainNodes->size(); id++)
      {
        auto node = trainNodes->get_train_node_id(id);
        if (node)
        {
          trainNodes->get_train_impl(node)->set_emergencystop();
        }
      }
      {
        OSMutexLock l(&lock_);
        remaining_ = config_cs_estop_packet_count();
        Singleton<SimpleUpdateLoop>::instance()->add_refresh_source(this
                                                                  , ESTOP_PRIORITY);
      }
    }
    else
    {
      OSMutexLock l(&lock_);
      remaining_ = 0;
      Singleton<SimpleUpdateLoop>::instance()->remove_refresh_source(this);
    }
  }

  void get_next_packet(unsigned code, Packet* packet)
  {
    packet->set_dcc_speed14(DccShortAddress(0), true, false
                          , Packet::EMERGENCY_STOP);
    {
      OSMutexLock l(&lock_);
      remaining_--;
      if (remaining_ <= 0)
      {
        Singleton<SimpleUpdateLoop>::instance()->remove_refresh_source(this);
      }
    }
  }

  Node *node() override
  {
    return node_;
  }

private:
  Node *node_;
  OSMutex lock_;
  int16_t remaining_;
  static constexpr unsigned ESTOP_PRIORITY = UpdateLoopBase::ESTOP_PRIORITY;
};

} // namespace esp32cs

#endif // ESTOP_HANDLER_H_
