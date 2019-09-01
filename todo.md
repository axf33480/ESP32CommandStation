# ESP32 Command Station Feature/Bug Tracking list
This document tracks features and bug fixes that are planned.

## v1.5.0
The primary focus of v1.5.0 is improvements to the DCC signal code and adding
RailCom support. The secondary focus is tighter integration with LCC.

### JMRI Interface

- [x] Replaced WiFiServer code with socket_listener from OpenMRNLite.

### General (misc)

- [x] Split up build_index_header.py into a common py module and PIO script.
- [x] Add PCB build types (base, OLED, LCD)
- [x] Replace ArduinoJson with "JSON for Modern C++"
- [-] Rework Roster Entry class to contain function id mappings.
- [x] Remove usage of log_X macros in favor of LOG.
- [x] Status LED output for WiFi, OPS and PROG, EXT_1 and EXT_2. EXT_1 and
  EXT_2 unused currently.

### Config

- [x] SoftAP support for initial config and "non-home" network.
- [x] Dynamic config for LCC and WiFi stored on SD/SPIFFS.
- [x] rename of WIFI_SSID to SSID_NAME and WIFI_PASSWORD to SSID_PASSWORD to be
  in sync with upcoming changes.
- [x] Split up monolithic json files to instead be individual files.

### DCC System

- [x] fix signal generation so it doesn't crash up when spi_flash disables
  cache.
- [x] remove hardware timer legacy code.
- [x] refactor signal generation to use: dcc::Packet, UpdateLoop, RailcomHub,
  ProgrammingTrackBackend, LocalTrackIf.
- [x] allow adjustment of the DCC preamble bit count, default is 16 (OPS) and
  22 (PROG). The OPS value is constrained between 11 and 20 and PROG between 22
  and 50.
- [ ] test OPS RailCom configuration.

### Web Interface

- [x] add dialog for failed CS requests.
- [x] Hide power button for prog track when it is off
- [x] Configure Station SSID/PW
- [ ] ESPAsyncWebServer can trigger heap corruption in the request processing
  when disconnecting the tcp client.
- [ ] WebSockets crash with ISR WDT, cause TBD.
- [x] Auto-connect WebSocket from initPage()
- [x] Remove overall power on/off as it no longer makes sense.
- [x] Add space in footer for clock so date/time are not smashed together.
- [x] Fixed S88 section hiding.
- [x] Fixed S88 sensor bus creation/edit json payload parameters.
- [x] Reintroduce websocket usage for pushing commands back to the CS.
- [x] auto-refresh of status pages
- [x] add busy/wait spinner for when data is loading (or being refreshed) in
  the web interface.

### LCC Integration

- [x] Integrate the WiFiConfiguration CDI element.
- [x] migrate to Esp32WiFiManager instead of Arduino WiFi library.
- [x] Refresh OpenMRNLite lib to latest openmrn code.
- [x] Add LCC metrics to the InfoScreen.
- [x] implement CV memory space.
- [x] Traction Protocol integration.
- [-] TrainSearch protocol.
- [x] CS CDI web interface.
- [x] CS Node ID reset from web interface.
- [x] Force factory reset when node id changes.

### InfoScreen

- [x] Move to StateFlow interface

### Nextion Interface

- [x] fix screen type detection.
- [x] lock to title screen until WiFi connects.
- [x] switch to timer based speed increment/decrement on the nextion mcu side.

### S88 Sensors

- [x] Add delay in s88SensorTask so that it gives time for other tasks to execute between updates.
- [x] restore custom web rendering of sensor data.

## Future planning:
The entries below are not tracked to a specific release or in any particular priority order.

### General

- [ ] CMake and VisualGDB support
- [ ] Report optional module failures via config portal.
- [ ] Reimplement Loco Consist leveraging LCC Traction Consist functionality.

### S88 Sensors

- [ ] Add S88 sensor data to InfoScreen status line, 16 sensor output rotation.

### DCC System

- [ ] Reimplement DCC Prog Track interface so it supports multiple requests (serialized).
- [ ] continue sending eStop packet until eStop is cleared.
- [ ] Concurrency guards for ProgrammingTrackBackend.

### Config
TBD

### Web Interface

- [ ] WiThrottle support (https://github.com/atanisoft/ESP32CommandStation/issues/15)
- [ ] Add strict validation of input parameter data.
- [ ] Expose Loco Consist creation.

### LCC Integration

- [ ] Broadcast events for turnout state change.
- [ ] Discard turnout events when turnout already in expected state (drop duplicate events).

### Nextion Interface

- [ ] auto turn on of track power from Nextion when interacting with loco/turnouts.
- [ ] add notification of turnout state change when changed external to the nextion code.
- [ ] replace Routes page with a Setup page which will include route creation.
- [ ] implement automatic resolver for component id during page initialization so we can drop component IDs from the argument list.
- [ ] move screen detection code into NeoNextion lib.
- [ ] add support for Nextion Upload via OTA in NeoNextion.
- [ ] correct screen size detection for 5" and 7" displays which send the same screen type prefix.

### OTA

- [ ] OTA support via JMRI / LCC.
- [ ] return to normal mode on Nextion when OTA fails.

### InfoScreen
TBD

### Documentation
No tasks have been added yet.
