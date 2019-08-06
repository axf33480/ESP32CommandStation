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

extern "C" {

void reboot()
{
  // shutdown and cleanup the configuration manager
  configStore.reset(nullptr);

  LOG(INFO, "Restarting ESP32 Command Station");
  // restart the node
  esp_restart();
}

ssize_t os_get_free_heap()
{
  return heap_caps_get_free_size(MALLOC_CAP_8BIT);
}

}
