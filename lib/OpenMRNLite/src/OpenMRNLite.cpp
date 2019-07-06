/** \copyright
 * Copyright (c) 2018, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
 * 
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \file OpenMRN.cpp
 * 
 * Implementation that needs to be compiled for the Arduino.
 *
 * @author Balazs Racz
 * @date 24 July 2018
 */

#include <OpenMRNLite.h>

namespace openmrn_arduino {

OpenMRN::OpenMRN(openlcb::NodeID node_id)
{
    init(node_id);
}


#ifdef ESP32
extern "C" {

#ifndef OPENMRN_EXCLUDE_REBOOT_IMPL
/// Reboots the ESP32 via the arduino-esp32 provided restart function.
void reboot()
{
    ESP.restart();
}
#endif

#ifndef OPENMRN_EXCLUDE_FREE_HEAP_IMPL
ssize_t os_get_free_heap()
{
    return ESP.getFreeHeap();
}
#endif

}
#endif // ESP32

} // namespace openmrn_arduino
