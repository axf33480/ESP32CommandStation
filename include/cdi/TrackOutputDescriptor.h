/**********************************************************************
DCC COMMAND STATION FOR ESP32

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

#ifndef _TRACK_OUTPUT_DESCRIPTOR_H_
#define _TRACK_OUTPUT_DESCRIPTOR_H_

#include <openlcb/ConfigRepresentation.hxx>

/// Track output configuration
CDI_GROUP(TrackOutputConfig);
CDI_GROUP_ENTRY(description,
                openlcb::StringConfigEntry<15>,
                Name("Description"),
                Description("Track output description."));
CDI_GROUP_ENTRY(event_short,
                openlcb::EventConfigEntry,
                Name("Short Detected"),
                Description("This event will be produced when a short has been detected on the track output."));
CDI_GROUP_ENTRY(event_short_cleared,
                openlcb::EventConfigEntry,
                Name("Short Cleared"),
                Description("This event will be produced when a short has been cleared on the track output."));
CDI_GROUP_ENTRY(event_shutdown,
                openlcb::EventConfigEntry,
                Name("H-Bridge Shutdown"),
                Description("This event will be produced when the track output power has exceeded the safety threshold of the H-Bridge."));
CDI_GROUP_ENTRY(event_shutdown_cleared,
                openlcb::EventConfigEntry,
                Name("H-Bridge Shutdown Cleared"),
                Description("This event will be produced when the track output power has returned to safe levels."));
CDI_GROUP_ENTRY(event_thermal_shutdown,
                openlcb::EventConfigEntry,
                Name("H-Bridge Thermal Shutdown"),
                Description("This event will be produced when the H-Bridge raises a thermal warning alert."));
CDI_GROUP_ENTRY(event_thermal_shutdown_cleared,
                openlcb::EventConfigEntry,
                Name("H-Bridge Thermal Shutdown Cleared"),
                Description("This event will be produced when the H-Bridge clears the thermal warning alert."));
CDI_GROUP_END();

#endif // _TRACK_OUTPUT_DESCRIPTOR_H_