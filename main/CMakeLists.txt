set(esp_idf_deps
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
)

set(required_deps
    "Configuration"
    "DCCppProtocol"
    "DCCSignalGenerator"
    "DCCTurnoutManager"
    "Esp32HttpServer"
    "LCCTrainSearchProtocol"
    "nlohmann_json"
    "OpenMRNLite"
    "TaskMonitor"
)

set(optional_deps
    "HC12"
    "NeoNextion"
    "StatusLED"
    "StatusDisplay"
    "GPIOOutputs"
    "GPIOSensors"
)

# it would be really nice to not have NeoNextion in the default dependencies
# but it appears that by the time cmake evaluates the CONFIG_NEXTION boolean
# flag the include args have already been set and can not be modified easily.

set(COMPONENT_REQUIRES "${esp_idf_deps} ${required_deps} ${optional_deps}")
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS
    "ESP32CommandStation.cpp"
    "ESP32TrainDatabase.cpp"
    "OpenMRNEsp32Overrides.cpp"
    "Interfaces/LocoNetInterface.cpp"
    "Interfaces/WebServer.cpp"
    "Interfaces/WiFiInterface.cpp"
    "Interfaces/nextion/NextionInterface.cpp"
    "Interfaces/nextion/NextionInterfaceAddressPage.cpp"
    "Interfaces/nextion/NextionInterfaceBasePage.cpp"
    "Interfaces/nextion/NextionInterfaceSetupPage.cpp"
    "Interfaces/nextion/NextionInterfaceThrottlePage.cpp"
    "Interfaces/nextion/NextionInterfaceTurnoutPage.cpp"
)

set(COMPONENT_ADD_INCLUDEDIRS ".")

register_component()

set_source_files_properties(ESP32CommandStation.cpp PROPERTIES COMPILE_FLAGS -Wno-implicit-fallthrough)
set_source_files_properties(Interfaces/WiFiInterface.cpp PROPERTIES COMPILE_FLAGS -Wno-implicit-fallthrough)
