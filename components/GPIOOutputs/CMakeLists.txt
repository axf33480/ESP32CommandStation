set(COMPONENT_SRCS
    "Outputs.cpp"
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