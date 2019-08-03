# ESP32 Command Station Feature/Bug Tracking list
This document tracks features and bug fixes that are planned.

## v1.4.0
The primary focus of v1.4.0 will be improvements to the DCC signal code and adding RailCom support.

### Updates from v1.3.0

### General (misc)

- [ ] CMake and VisualGDB support (https://github.com/atanisoft/ESP32CommandStation/pull/22)
- [x] Split up build_index_header.py into a common py module and PIO script.
- [ ] Remove usages of String and __FlashStringHelper.
- [x] Add PCB build types (base, OLED, LCD)

### Config

- [-] SoftAP support for initial config and "non-home" network. (https://github.com/atanisoft/ESP32CommandStation/issues/4)

### DCC System

- [-] continue sending eStop packet until eStop is cleared.
- [x] refactor signal generation to use: dcc::Packet, UpdateLoop, RailcomHub, ProgrammingTrackBackend, LocalTrackIf.
- [x] allow adjustment of the DCC preamble bit count, default is 16 (OPS) and 22 (PROG). The OPS value is constrained between 11 and 20 and PROG between 22 and 50.
- [ ] test and expose OPS RailCom configuration.

### Web Interface

- [ ] add dialog for failed CS requests.
- [ ] Expose Loco Consist creation.
- [x] Hide power button for prog track when it is off

### LCC Integration

- [x] migrate to Esp32WiFiManager instead of Arduino WiFi library.
- [x] Refresh OpenMRNLite lib to latest openmrn code.
- [-] adjust InfoScreen LCC details so they are useful, right now it is a placeholder.
- [x] implement CV memory space.

### InfoScreen

- [x] Move to StateFlow interface

## Future planning:
The entries below are not tracked to a specific release or in any particular priority order.

### S88 Sensors

- [ ] Add S88 sensor data to InfoScreen status line, 16 sensor output rotation.

### DCC System

- [-] rework DCC Prog Track interface so it supports multiple requests (serialized) and async response to web.
- [ ] Concurrency guards for ProgrammingTrackBackend.

### Config

- [ ] Dynamic WiFi configuration support (multiple SSID, strength based selection of SSID, etc)
- [ ] Dynamic command station feature support (S88, OLED, LCD, LocoNet, Nextion, LCC, etc)

### Web Interface

- [ ] WiThrottle support (https://github.com/atanisoft/ESP32CommandStation/issues/15)
- [ ] Add strict validation of input parameter data.
- [ ] Rework web prog req to be async rather than blocking

### LCC Integration

- [ ] Traction proxy impl.
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
