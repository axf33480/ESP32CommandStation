set(deps
    "driver"
    "esp_adc_cal"
    "efuse"
    "soc"
    "nvs_flash"
    "spi_flash"
    "vfs"
    "esp_common"
    "bootloader_support"
    "app_update"
    "spiffs"
    "fatfs"
    "esp_wifi"
    "nlohmann_json"
    "OpenMRNLite"
    "LCCTrainSearchProtocol"
    "NeoPixelBus"
    "Esp32HttpServer"
    "NeoNextion"
)

# it would be really nice to not have NeoNextion in the default dependencies
# but it appears that by the time cmake evaluates the CONFIG_NEXTION boolean
# flag the include args have already been set and can not be modified easily.

set(srcs
    "."
    "DCC"
    "Interfaces"
    "Interfaces/nextion"
    "IO"
    "stateflows"
)

set(COMPONENT_REQUIRES "${deps}")
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCDIRS "${srcs}")

set(COMPONENT_ADD_INCLUDEDIRS ".")

register_component()

set_source_files_properties(ESP32CommandStation.cpp PROPERTIES COMPILE_FLAGS -Wno-implicit-fallthrough)
set_source_files_properties(ConfigurationManager.cpp PROPERTIES COMPILE_FLAGS -Wno-implicit-fallthrough)
set_source_files_properties(Interfaces/WiFiInterface.cpp PROPERTIES COMPILE_FLAGS -Wno-implicit-fallthrough)
