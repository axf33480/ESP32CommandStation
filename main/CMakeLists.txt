set(deps
    "driver"
    "efuse"
    "soc"
    "nvs_flash"
    "spi_flash"
    "vfs"
    "esp_common"
    "esp_adc_cal"
    "bootloader_support"
    "app_update"
    "esp_wifi"
    "nlohmann_json"
    "OpenMRNLite"
    "LCCTrainSearchProtocol"
    "Esp32HttpServer"
    "NeoNextion"
    "EStopGenerator"
    "HBridge"
    "StatusLED"
    "StatusDisplay"
    "TaskMonitor"
    "DCCSignalGenerator"
    "DCCppProtocol"
    "Configuration"
    "DCCTurnoutManager"
    "HC12"
)

# it would be really nice to not have NeoNextion in the default dependencies
# but it appears that by the time cmake evaluates the CONFIG_NEXTION boolean
# flag the include args have already been set and can not be modified easily.

set(srcs
    "."
    "Interfaces"
    "Interfaces/nextion"
    "IO"
)

set(COMPONENT_REQUIRES "${deps}")
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCDIRS "${srcs}")

set(COMPONENT_ADD_INCLUDEDIRS ".")

register_component()

set_source_files_properties(ESP32CommandStation.cpp PROPERTIES COMPILE_FLAGS -Wno-implicit-fallthrough)
set_source_files_properties(Interfaces/WiFiInterface.cpp PROPERTIES COMPILE_FLAGS -Wno-implicit-fallthrough)
