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

set_source_files_properties(EStopHandler.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(LocalTrackIf.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(MonitoredHBridge.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)
set_source_files_properties(RMTTrackDevice.cpp PROPERTIES COMPILE_FLAGS -Wno-ignored-qualifiers)