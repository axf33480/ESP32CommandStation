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

#ifndef AUTO_SYNC_FILE_FLOW_H_
#define AUTO_SYNC_FILE_FLOW_H_

#include <executor/Service.hxx>
#include <executor/StateFlow.hxx>

/// Simple state flow to configure automatic calls to fsync on a single file
/// handle at regular intervals.
class AutoSyncFileFlow : public StateFlowBase
{
public:
  /// Constructor
  ///
  /// @param service is the @ref Service to hook into for periodic callbacks.
  /// @param sync_fd is the file handle to sync.
  /// @param interval is the interval at which to sync the file handle. Default
  /// is once per second.
  AutoSyncFileFlow(Service *service
                 , int sync_fd
                 , uint64_t interval=SEC_TO_NSEC(1))
                 : StateFlowBase(service)
                 , fd_(sync_fd)
                 , syncInterval_(interval)
                 , syncName_(StringPrintf("AutoSyncFileFlow(%d)", fd_))
  {
    HASSERT(fd_ >= 0);
    HASSERT(syncInterval_ > 0);
    start_flow(STATE(sleep_and_call_sync));
  }

private:
  const int fd_;
  const uint64_t syncInterval_;
  const std::string syncName_;
  StateFlowTimer timer_{this};

  Action sleep_and_call_sync()
  {
    return sleep_and_call(&timer_, syncInterval_, STATE(sync));
  }

  Action sync()
  {
    ERRNOCHECK(syncName_.c_str(), fsync(fd_));
    return call_immediately(STATE(sleep_and_call_sync));
  }
};

#endif // AUTO_SYNC_FILE_FLOW_H_