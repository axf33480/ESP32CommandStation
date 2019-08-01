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

#ifndef CS_CDI_H_
#define CS_CDI_H_

#include <openlcb/ConfigRepresentation.hxx>
#include <openlcb/MemoryConfig.hxx>
#include <openlcb/TractionCvCdi.hxx>
#include <freertos_drivers/esp32/Esp32WiFiConfiguration.hxx>

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

namespace openlcb
{
    using TrackOutputs = RepeatedGroup<TrackOutputConfig, 2>;

    /// Defines the main segment in the configuration CDI. This is laid out at
    /// origin 128 to give space for the ACDI user data at the beginning.
    CDI_GROUP(CommandStationSegment, Segment(MemoryConfigDefs::SPACE_CONFIG),
              Offset(128));
    /// Each entry declares the name of the current entry, then the type and
    /// then optional arguments list.
    CDI_GROUP_ENTRY(internal_config, InternalConfigData);
    /// CV Access via MemoryConfig protocol.
    CDI_GROUP_ENTRY(cv, TractionShortCvSpace);
    /// WiFi configuration
    CDI_GROUP_ENTRY(wifi, WiFiConfiguration, Name("WiFi Configuration"));
    /// H-Bridge configuration
    CDI_GROUP_ENTRY(hbridge, TrackOutputs, Name("H-Bridge Configuration"));
    CDI_GROUP_END();

    /// This segment is only needed temporarily until there is program code to set
    /// the ACDI user data version byte.
    CDI_GROUP(VersionSeg, Segment(MemoryConfigDefs::SPACE_CONFIG),
        Name("Version information"));
    CDI_GROUP_ENTRY(acdi_user_version, Uint8ConfigEntry,
        Name("ACDI User Data version"),
        Description("Set to 2 and do not change."));
    CDI_GROUP_END();

    /// The main structure of the CDI. ConfigDef is the symbol used in
    /// src/LCCInterface.cpp to refer to the configuration defined here.
    CDI_GROUP(ConfigDef, MainCdi());
    /// Adds the <identification> tag with the values from SNIP_STATIC_DATA
    /// above.
    CDI_GROUP_ENTRY(ident, Identification);
    /// Adds an <acdi> tag.
    CDI_GROUP_ENTRY(acdi, Acdi);
    /// Adds a segment for changing the values in the ACDI user-defined
    /// space. UserInfoSegment is defined in the system header.
    CDI_GROUP_ENTRY(userinfo, UserInfoSegment);
    /// Adds the main configuration segment.
    CDI_GROUP_ENTRY(seg, CommandStationSegment);
    /// Adds the versioning segment.
    CDI_GROUP_ENTRY(version, VersionSeg);
    CDI_GROUP_END();
}

#endif // CS_CDI_H_