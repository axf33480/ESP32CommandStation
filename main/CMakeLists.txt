set(COMPONENT_REQUIRES driver esp_adc_cal efuse soc nvs_flash spi_flash vfs esp_common bootloader_support app_update spiffs fatfs esp_wifi
                       nlohmann_json OpenMRNLite LCCTrainSearchProtocol NeoPixelBus Esp32HttpServer)
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCDIRS ". DCC Interfaces IO stateflows")

set(COMPONENT_ADD_INCLUDEDIRS ".")

register_component()

set_source_files_properties(ESP32CommandStation.cpp PROPERTIES COMPILE_FLAGS -Wno-implicit-fallthrough)
set_source_files_properties(ConfigurationManager.cpp PROPERTIES COMPILE_FLAGS -Wno-implicit-fallthrough)
set_source_files_properties(Interfaces/WiFiInterface.cpp PROPERTIES COMPILE_FLAGS -Wno-implicit-fallthrough)
