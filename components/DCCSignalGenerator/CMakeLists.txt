set(COMPONENT_ADD_INCLUDEDIRS "include" )

set(COMPONENT_SRCS
    "EStopHandler.cpp"
    "LocalTrackIf.cpp"
    "MonitoredHBridge.cpp"
    "RMTTrackDevice.cpp"
)

set(COMPONENT_REQUIRES
    "OpenMRNLite"
    "driver"
    "esp_adc_cal"
    "nlohmann_json"
    "StatusDisplay"
    "StatusLED"
    "LCCTrainSearchProtocol"
)

register_component()