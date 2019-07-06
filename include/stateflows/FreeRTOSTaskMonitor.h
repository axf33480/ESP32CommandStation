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

#include "ESP32CommandStation.h"

#include <executor/StateFlow.hxx>
#include <openlcb/SimpleStack.hxx>

class FreeRTOSTaskMonitor : public StateFlowBase {
public:
   FreeRTOSTaskMonitor(openlcb::SimpleCanStack *stack)
    : StateFlowBase(stack->service())
   {
#if configUSE_TRACE_FACILITY
    start_flow(STATE(delay));
#endif
   }
private:
  StateFlowTimer timer_{this};
  const uint64_t reportInterval_{SEC_TO_NSEC(60)};    // 1min
  const uint64_t taskListInterval_{SEC_TO_USEC(300)}; // 5min
  uint64_t lastTaskList_{0};

  Action report()
  {
    uint64_t now = esp_timer_get_time();
    UBaseType_t taskCount = uxTaskGetNumberOfTasks();
    LOG(INFO,
        "[TaskMon] uptime: %02d:%02d:%02d freeHeap: %u, largest free block: %u, tasks: %d"
      , (uint32_t)(USEC_TO_SEC(esp_timer_get_time()) / 3600)
      , (uint32_t)(USEC_TO_SEC(esp_timer_get_time()) % 3600) / 60
      , (uint32_t)(USEC_TO_SEC(esp_timer_get_time()) - (USEC_TO_SEC(esp_timer_get_time()) % 3600))
      , heap_caps_get_free_size(MALLOC_CAP_INTERNAL)
      , heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)
      , taskCount
    );
    if ((now - lastTaskList_) > taskListInterval_ || !lastTaskList_)
    {
      std::unique_ptr<TaskStatus_t[]> taskList(new TaskStatus_t[taskCount]);
      uint32_t ulTotalRunTime;
      UBaseType_t retrievedTaskCount = uxTaskGetSystemState(taskList.get(),
                                                            taskCount,
                                                            &ulTotalRunTime);
      for (int task = 0; task < retrievedTaskCount; task++)
      {
        LOG(INFO,
            "[TaskMon] %-16s id:%3d, prio:%2d/%2d, stack:%5d, core:%4s, "
            "cpu%%:%6.2f, state:%s"
          , taskList[task].pcTaskName
          , taskList[task].xTaskNumber
          , taskList[task].uxCurrentPriority
          , taskList[task].uxBasePriority
          , taskList[task].usStackHighWaterMark
          , taskList[task].xCoreID == tskNO_AFFINITY ? "BOTH" :
            taskList[task].xCoreID == PRO_CPU_NUM ? "PRO" :
            taskList[task].xCoreID == APP_CPU_NUM ? "APP" : "UNK"
          , ((float)taskList[task].ulRunTimeCounter / (float)ulTotalRunTime) * 100.0f
          , taskList[task].eCurrentState == eRunning ? "Running" :
            taskList[task].eCurrentState == eReady ? "Ready" :
            taskList[task].eCurrentState == eBlocked ? "Blocked" :
            taskList[task].eCurrentState == eSuspended ? "Suspended" :
            taskList[task].eCurrentState == eDeleted ? "Deleted" : "Unknown"
        );
      }
      lastTaskList_ = now;
    }
    return call_immediately(STATE(delay));
  }
  Action delay()
  {
    return sleep_and_call(&timer_, reportInterval_, STATE(report));
  }
};