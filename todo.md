# ESP32 Command Station Feature/Bug Tracking list
This document tracks features and bug fixes that are planned.

## v1.5.0
The primary focus of v1.5.0 is improvements to the DCC signal code, adding
RailCom support, tighter LCC integration and general stability improvements.

-   [x] JMRI: Replaced WiFiServer code with socket_listener from OpenMRNLite.
-   [ ] JMRI: Convert to StateFlow pattern.
-   [x] Split up build_index_header.py into a common py module and PIO script.
-   [x] Add PCB build types (base, OLED, LCD)
-   [x] Replace ArduinoJson with "JSON for Modern C++"
-   [ ] Rework Roster Entry class to contain function id mappings.
-   [x] Remove usage of log_X macros in favor of LOG.
-   [x] Status LED output for WiFi, OPS and PROG, EXT_1 and EXT_2. EXT_1 and EXT_2 unused currently.
-   [x] Config: SoftAP support for initial config and "non-home" network.
-   [x] Config: Dynamic config for LCC and WiFi stored on SD/SPIFFS.
-   [x] Config: rename of WIFI_SSID to SSID_NAME and WIFI_PASSWORD to SSID_PASSWORD to be in sync with upcoming changes.
-   [x] Config: Split up monolithic json files to instead be individual files.
-   [x] DCC: fix signal generation so it doesn't crash up when spi_flash disables cache.
-   [x] DCC: remove hardware timer legacy code.
-   [x] DCC: refactor signal generation to use: dcc::Packet, UpdateLoop, RailcomHub, ProgrammingTrackBackend, LocalTrackIf.
-   [x] DCC: allow adjustment of the DCC preamble bit count, default is 16 (OPS) and 22 (PROG). The OPS value is constrained between 11 and 20 and PROG between 22 and 50.
-   [ ] DCC: test OPS RailCom configuration.
-   [x] Web: Replace webserver code with StateFlow based server.
-   [x] Web: add dialog for failed CS requests.
-   [x] Web: Hide power button for prog track when it is off.
-   [x] Web: Configure Station SSID/PW
-   [x] Web: Auto-connect WebSocket from initPage()
-   [x] Web: Remove overall power on/off as it no longer makes sense.
-   [x] Web: Add space in footer for clock so date/time are not smashed together.
-   [x] Web: Fixed S88 section hiding.
-   [x] Web: Fixed S88 sensor bus creation/edit json payload parameters.
-   [x] Web: Reintroduce websocket usage for pushing commands back to the CS.
-   [x] Web: auto-refresh of status pages
-   [x] Web: add busy/wait spinner for when data is loading (or being refreshed) in the web interface.
-   [x] LCC: Integrate the WiFiConfiguration CDI element.
-   [x] LCC: migrate to Esp32WiFiManager instead of Arduino WiFi library.
-   [x] LCC: Refresh OpenMRNLite lib to latest openmrn code.
-   [x] LCC: Add LCC metrics to the InfoScreen.
-   [x] LCC: implement CV memory space.
-   [x] LCC: Traction Protocol integration.
-   [-] LCC: TrainSearch protocol.
-   [x] LCC: CS CDI web interface.
-   [x] LCC: CS Node ID reset from web interface.
-   [x] LCC: Force factory reset when node id changes.
-   [x] Move InfoScreen to StateFlow interface
-   [x] Nextion: fix screen type detection.
-   [x] Nextion: lock to title screen until WiFi connects.
-   [x] Nextion: switch to timer based speed increment/decrement on the nextion mcu side.
-   [ ] Nextion: Switch to StateFlow interface
-   [x] S88: Add delay in s88SensorTask so that it gives time for other tasks to execute between updates.
-   [x] S88: restore custom web rendering of sensor data.

## Future planning
The entries below are not tracked to a specific release or in any particular priority order.

-   [ ] CMake and VisualGDB support
-   [ ] Report optional module failures via config portal.
-   [ ] Reimplement Loco Consist leveraging LCC Traction Consist functionality.
-   [ ] S88: Add S88 sensor data to InfoScreen status line, 16 sensor output rotation.
-   [ ] DCC: Reimplement DCC Prog Track interface so it supports multiple requests (serialized).
-   [ ] DCC: continue sending eStop packet until eStop is cleared.
-   [ ] DCC: Concurrency guards for ProgrammingTrackBackend.
-   [ ] WiThrottle support (https://github.com/atanisoft/ESP32CommandStation/issues/15)
-   [ ] Web: Add strict validation of input parameter data.
-   [ ] Web: Expose Loco Consist creation.
-   [ ] LCC: Broadcast events for turnout state change.
-   [ ] LCC: Discard turnout events when turnout already in expected state (drop duplicate events).
-   [ ] Nextion: auto turn on of track power from Nextion when interacting with loco/turnouts.
-   [ ] Nextion: add notification of turnout state change when changed external to the nextion code.
-   [ ] Nextion: replace Routes page with a Setup page which will include route creation.
-   [ ] Nextion: implement automatic resolver for component id during page initialization so we can drop component IDs from the argument list.
-   [ ] Nextion: move screen detection code into NeoNextion lib.
-   [ ] Nextion: add support for Nextion Upload via OTA in NeoNextion.
-   [ ] Nextion: correct screen size detection for 5" and 7" displays which send the same screen type prefix.
-   [ ] OTA: OTA support via JMRI / LCC.
-   [ ] OTA: return to normal mode on Nextion when OTA fails.
