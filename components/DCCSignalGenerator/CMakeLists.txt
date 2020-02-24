set(COMPONENT_SRCDIRS "src" )

set(COMPONENT_ADD_INCLUDEDIRS "include" )

set(COMPONENT_REQUIRES
    "OpenMRNLite"
    "driver"
    "esp_adc_cal"
    "HBridge"
    "nlohmann_json"
    "StatusDisplay"
    "StatusLED"
)

register_component()