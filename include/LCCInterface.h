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

#pragma once

#include <executor/StateFlow.hxx>

class LCCInterface {
public:
  LCCInterface();
  void init();
  void update();
  void processWiFiEvent(system_event_id_t event);
  size_t getNodeAliasCount();
};

class InfoScreenStatCollector : public StateFlowBase {
public:
    InfoScreenStatCollector(openlcb::SimpleCanStack *stack) : StateFlowBase(stack->service()), stack_(stack) {
        start_flow(STATE(delay));
    }
    StateFlowBase::Action delay() {
        return sleep_and_call(&timer_, SEC_TO_NSEC(5), STATE(update_count));
    }
    StateFlowBase::Action update_count() {
        // TODO: adjust this to capture actual node counts rather than just cache size
        remoteNodeCount_ = stack_->iface()->remote_aliases()->size();
        localNodeCount_ = stack_->iface()->local_aliases()->size();
        executorCount_ = stack_->service()->executor()->sequence();
        return call_immediately(STATE(delay));
    }
    size_t getRemoteNodeCount() const {
        return remoteNodeCount_;
    }
    size_t getLocalNodeCount() const {
        return localNodeCount_;
    }
    uint32_t getExecutorCount() const {
        return executorCount_;
    }
private:
    openlcb::SimpleCanStack *stack_;
    StateFlowTimer timer_{this};
    size_t remoteNodeCount_{0};
    size_t localNodeCount_{0};
    uint32_t executorCount_{0};
};

extern LCCInterface lccInterface;
extern InfoScreenStatCollector infoScreenCollector;
