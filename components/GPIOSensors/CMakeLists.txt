set(COMPONENT_SRCS
    "src/Sensors.cpp"
    "src/RemoteSensors.cpp"
    "src/S88Sensors.cpp"
)

set(COMPONENT_ADD_INCLUDEDIRS "include" )

set(COMPONENT_REQUIRES
    "Configuration"
    "DCCppProtocol"
    "nlohmann_json"
    "OpenMRNLite"
    "StatusDisplay"
    "driver"
)

register_component()