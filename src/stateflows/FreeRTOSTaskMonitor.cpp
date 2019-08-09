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
  // explicit cast is necessary for these next two lines due to compiler
  // warnings for data type truncation.
  , reportInterval_{(uint64_t)SEC_TO_NSEC(config_cs_task_list_report_interval_sec())}
  , taskListInterval_{(uint64_t)SEC_TO_USEC(config_cs_task_list_stats_interval_sec())}
{
#if configUSE_TRACE_FACILITY
  start_flow(STATE(delay));
#endif
}

StateFlowBase::Action FreeRTOSTaskMonitor::report()
{
  vector<TaskStatus_t> taskList;
  uint32_t ulTotalRunTime{0};
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
  // exit early if we do not need to report task state
  if (config_cs_task_list_report() == CONSTANT_FALSE)
  {
    return call_immediately(STATE(delay));
  }
  uint64_t now = esp_timer_get_time();
  if ((now - lastTaskList_) > taskListInterval_ || !lastTaskList_)
  {
    taskList.resize(taskCount);
    UBaseType_t retrievedTaskCount = uxTaskGetSystemState(&taskList[0],
                                                          taskCount,
                                                          &ulTotalRunTime);
    // adjust this time so we can use it for percentages.
    ulTotalRunTime /= 100;
    // sort by runtime
    std::sort(taskList.begin(), taskList.end(),
    [](const TaskStatus_t &left, const TaskStatus_t &right)
    {
      return left.ulRunTimeCounter > right.ulRunTimeCounter;
    });
    for (int task = 0; task < retrievedTaskCount; task++)
    {
      LOG(INFO,
          "[TaskMon] %-16s id:%3d, priority:%2d/%2d, free stack:%5d, core:%4s, "
          "cpu%%:%-3d, state:%s"
        , taskList[task].pcTaskName
        , taskList[task].xTaskNumber
        , taskList[task].uxCurrentPriority
        , taskList[task].uxBasePriority
        , taskList[task].usStackHighWaterMark
        , taskList[task].xCoreID == tskNO_AFFINITY ? "BOTH" :
          taskList[task].xCoreID == PRO_CPU_NUM ? "PRO" :
          taskList[task].xCoreID == APP_CPU_NUM ? "APP" : "UNK"
        , (taskList[task].ulRunTimeCounter / portNUM_PROCESSORS) / ulTotalRunTime
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