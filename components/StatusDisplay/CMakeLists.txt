set(COMPONENT_SRCS
    "StatusDisplay.cpp"
)

set(COMPONENT_ADD_INCLUDEDIRS "include" )

set(COMPONENT_REQUIRES
    "app_update"
    "Configuration"
    "DCCSignalGenerator"
    "OpenMRNLite"
    "LCCTrainSearchProtocol"
)

register_component()