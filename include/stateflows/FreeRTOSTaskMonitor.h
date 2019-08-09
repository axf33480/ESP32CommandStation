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

#ifndef FREERTOS_TASK_MON_H_
#define FREERTOS_TASK_MON_H_

#include <executor/Service.hxx>
#include <executor/StateFlow.hxx>
#include <openlcb/SimpleStack.hxx>

class FreeRTOSTaskMonitor : public StateFlowBase {
public:
  FreeRTOSTaskMonitor(Service *);

private:
  StateFlowTimer timer_{this};
  const uint64_t reportInterval_;
  const uint64_t taskListInterval_;
  uint64_t lastTaskList_{0};

  STATE_FLOW_STATE(report);

  Action delay()
  {
    return sleep_and_call(&timer_, reportInterval_, STATE(report));
  }
};
#endif // FREERTOS_TASK_MON_H_