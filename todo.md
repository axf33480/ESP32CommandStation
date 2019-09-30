# ESP32 Command Station Feature/Bug Tracking list
This document tracks features and bug fixes that are planned.

## v1.5.0
The primary focus of v1.5.0 is improvements to the DCC signal code, adding
RailCom support, tighter LCC integration and general stability improvements.

-   [x] Build: Split up build_index_header.py into a common py module and PIO script.
-   [x] Build: Add PCB build types (base, OLED, LCD)
-   [x] Config: Dynamic config for LCC and WiFi stored on SD/SPIFFS.
-   [x] Config: rename of WIFI_SSID to SSID_NAME and WIFI_PASSWORD to SSID_PASSWORD to be in sync with upcoming changes.
-   [x] Config: SoftAP support for initial config and "non-home" network.
-   [x] Config: Split up monolithic json files to instead be individual files.
-   [x] DCC: allow adjustment of the DCC preamble bit count, default is 16 (OPS) and 22 (PROG). The OPS value is constrained between 11 and 20 and PROG between 22 and 50.
-   [x] DCC: fix signal generation so it doesn't crash up when spi_flash disables cache.
-   [x] DCC: remove hardware timer legacy code.
-   [x] DCC: refactor signal generation to use: dcc::Packet, UpdateLoop, RailcomHub, ProgrammingTrackBackend, LocalTrackIf.
-   [ ] DCC: test OPS RailCom configuration.
-   [ ] JMRI: Convert to StateFlow pattern.
-   [x] JMRI: Replaced WiFiServer code with socket_listener from OpenMRNLite.
-   [x] LCC: Add LCC metrics to the InfoScreen.
-   [x] LCC: CS CDI web interface.
-   [x] LCC: CS Node ID reset from web interface.
-   [x] LCC: Force factory reset when node id changes.
-   [x] LCC: Integrate CV memory space.
-   [x] LCC: Integrate the WiFiConfiguration CDI element.
-   [x] LCC: migrate to Esp32WiFiManager instead of Arduino WiFi library.
-   [x] LCC: Traction Protocol integration.
-   [-] LCC: TrainSearch protocol.
-   [ ] Misc: Rework Roster Entry class to contain function id mappings.
-   [x] Misc: Remove usage of log_X macros in favor of LOG.
-   [x] Nextion: fix screen type detection.
-   [x] Nextion: lock to title screen until WiFi connects.
-   [x] Nextion: switch to timer based speed increment/decrement on the nextion mcu side.
-   [x] Status: Move InfoScreen to StateFlow interface
-   [x] Status: Status LED output for WiFi, OPS and PROG, EXT_1 and EXT_2. EXT_1 and EXT_2 unused currently.
-   [x] S88: Add delay in s88SensorTask so that it gives time for other tasks to execute between updates.
-   [x] S88: restore custom web rendering of sensor data.
-   [x] Web: add busy/wait spinner for when data is loading (or being refreshed) in the web interface.
-   [x] Web: add dialog for failed CS requests.
-   [x] Web: Add space in footer for clock so date/time are not smashed together.
-   [x] Web: Auto-connect WebSocket from initPage()
-   [x] Web: auto-refresh of status pages
-   [x] Web: Configure Station SSID/PW
-   [x] Web: Fixed S88 section hiding.
-   [x] Web: Fixed S88 sensor bus creation/edit json payload parameters.
-   [x] Web: Hide power button for prog track when it is off.
-   [x] Web: Reintroduce websocket usage for pushing commands back to the CS.
-   [x] Web: Replace ArduinoJson with "JSON for Modern C++"
-   [x] Web: Replace webserver code with StateFlow based server.
-   [x] Web: Remove overall power on/off as it no longer makes sense.
-   [x] Web: Implement support for application/x-www-form-urlencoded POST/PUT data.
-   [ ] Web: Auto-refresh tables when delete/edit completes.

## Future planning
The entries below are not tracked to a specific release or in any particular priority order.

-   [ ] Build: CMake and VisualGDB support
-   [ ] DCC: Concurrency guards for ProgrammingTrackBackend.
-   [ ] DCC: continue sending eStop packet until eStop is cleared.
-   [ ] DCC: Reimplement DCC Prog Track interface so it supports multiple requests (serialized).
-   [ ] LCC: Broadcast events for turnout state change.
-   [ ] LCC: Discard turnout events when turnout already in expected state (drop duplicate events).
-   [ ] LCC: Reimplement Loco Consist leveraging LCC Traction Consist functionality.
-   [ ] Misc: WiThrottle support (https://github.com/atanisoft/ESP32CommandStation/issues/15)
-   [ ] Nextion: add support for Nextion Upload via OTA in NeoNextion.
-   [ ] Nextion: add notification of turnout state change when changed external to the nextion code.
-   [ ] Nextion: auto turn on of track power from Nextion when interacting with loco/turnouts.
-   [ ] Nextion: adjust screen size detection for 5" and 7" displays which send the same screen type prefix.
-   [ ] Nextion: implement automatic resolver for component id during page initialization so we can drop component IDs from the argument list.
-   [ ] Nextion: move screen detection code into NeoNextion lib.
-   [ ] Nextion: move to StateFlow interface
-   [ ] Nextion: replace Routes page with a Setup page which will include route creation.
-   [ ] OTA: OTA support via JMRI / LCC.
-   [ ] OTA: return to normal mode on Nextion when OTA fails.
-   [ ] S88: Add S88 sensor data to InfoScreen status line, 16 sensor output rotation.
-   [ ] Web: Add strict validation of input parameter data.
-   [ ] Web: Expose Loco Consist creation.
-   [ ] Web: Report optional module failures via config portal.
