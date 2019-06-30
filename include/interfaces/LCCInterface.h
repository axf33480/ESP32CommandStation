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

#include <dcc/RailcomHub.hxx>
#include <executor/StateFlow.hxx>

class LCCInterface {
public:
  LCCInterface();
  void init();
  void update();
};

class InfoScreenStatCollector : public StateFlowBase {
public:
    InfoScreenStatCollector(openlcb::SimpleCanStack *stack) : StateFlowBase(stack->service()), stack_(stack) {
        start_flow(STATE(startup_delay));
    }
    StateFlowBase::Action startup_delay() {
        return sleep_and_call(&timer_, SEC_TO_NSEC(5), STATE(update_count));
    }
    StateFlowBase::Action update_count() {
        // TODO: Migrate to DG listener to capture node aliases as they are announced.
        remoteNodeCount_ = static_cast<openlcb::IfCan *>(stack_->iface())->remote_aliases()->size();
        localNodeCount_ = static_cast<openlcb::IfCan *>(stack_->iface())->local_aliases()->size();
        uint32_t currentCount = stack_->service()->executor()->sequence();
        executorCount_ = currentCount - lastExecutorCount_;
        lastExecutorCount_ = currentCount;
        return sleep_and_call(&timer_, SEC_TO_NSEC(5), STATE(update_count));
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
    size_t getPoolFreeCount() const {
        return stack_->can_hub()->pool()->free_items();
    }
    size_t getPoolSize() const {
        return stack_->can_hub()->pool()->total_size();
    }
    uint32_t getDatagramCount() const {
        return stack_->dg_service()->client_allocator()->pending();
    }
private:
    openlcb::SimpleCanStack *stack_;
    StateFlowTimer timer_{this};
    size_t remoteNodeCount_{0};
    size_t localNodeCount_{0};
    uint32_t executorCount_{0};
    uint32_t lastExecutorCount_{0};
};

extern LCCInterface lccInterface;
extern InfoScreenStatCollector infoScreenCollector;
extern dcc::RailcomHubFlow railComHub;
