set(COMPONENT_SRCDIRS "src" )

set(COMPONENT_ADD_INCLUDEDIRS "include" )

set(COMPONENT_REQUIRES
    "OpenMRNLite"
    "driver"
    "esp_adc_cal"
    "StatusLED"
    "nlohmann_json"
)

register_component()