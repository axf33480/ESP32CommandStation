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

FreeRTOSTaskMonitor::FreeRTOSTaskMonitor(Service *service)
  : StateFlowBase(service)
{
#if configUSE_TRACE_FACILITY
  start_flow(STATE(delay));
#endif
}

StateFlowBase::Action FreeRTOSTaskMonitor::report()
{
  UBaseType_t taskCount = uxTaskGetNumberOfTasks();
  LOG(INFO,
      "[TaskMon] uptime: %02d:%02d:%02d freeHeap: %u, largest free block: %u, tasks: %d"
    , (uint32_t)(USEC_TO_SEC(esp_timer_get_time()) / 3600)
    , (uint32_t)(USEC_TO_SEC(esp_timer_get_time()) % 3600) / 60
    , (uint32_t)(USEC_TO_SEC(esp_timer_get_time()) % 60 )
    , heap_caps_get_free_size(MALLOC_CAP_INTERNAL)
    , heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)
    , taskCount
  );
#if ENABLE_TASK_LIST_REPORTING
  uint64_t now = esp_timer_get_time();
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
          "cpu%%:%-3d, state:%s"
        , taskList[task].pcTaskName
        , taskList[task].xTaskNumber
        , taskList[task].uxCurrentPriority
        , taskList[task].uxBasePriority
        , taskList[task].usStackHighWaterMark
        , taskList[task].xCoreID == tskNO_AFFINITY ? "BOTH" :
          taskList[task].xCoreID == PRO_CPU_NUM ? "PRO" :
          taskList[task].xCoreID == APP_CPU_NUM ? "APP" : "UNK"
        , taskList[task].ulRunTimeCounter / (ulTotalRunTime / 100)
        , taskList[task].eCurrentState == eRunning ? "Running" :
          taskList[task].eCurrentState == eReady ? "Ready" :
          taskList[task].eCurrentState == eBlocked ? "Blocked" :
          taskList[task].eCurrentState == eSuspended ? "Suspended" :
          taskList[task].eCurrentState == eDeleted ? "Deleted" : "Unknown"
      );
    }
    lastTaskList_ = now;
  }
#endif // ENABLE_TASK_LIST_REPORTING
  return call_immediately(STATE(delay));
}