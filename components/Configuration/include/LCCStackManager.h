/**********************************************************************
ESP32 COMMAND STATION

COPYRIGHT (c) 2020 Mike Dunston

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

#include "CSConfigDescriptor.h"

#include <utils/Singleton.hxx>

namespace openlcb
{
  class SimpleStackBase;
}

class AutoSyncFileFlow;

namespace esp32cs
{

class LCCStackManager : public Singleton<LCCStackManager>
{
public:
  LCCStackManager(const esp32cs::Esp32ConfigDef &cfg);
  openlcb::SimpleStackBase *stack();
  void start(bool is_sd);
  void shutdown();
  bool set_node_id(std::string);
  bool reconfigure_can(bool enable);
  void factory_reset();
  std::string get_config_json();
private:
  const Esp32ConfigDef cfg_;
  int fd_;
  uint64_t nodeID_;
  openlcb::SimpleStackBase *stack_;
  Executable *canBridge_;
  Executable *can_;
  AutoSyncFileFlow *configAutoSync_;
};

} // namespace esp32cs