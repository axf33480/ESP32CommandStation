set(COMPONENT_SRCS
    "Turnouts.cpp"
)

set(COMPONENT_ADD_INCLUDEDIRS "include" )

set(COMPONENT_REQUIRES
    "OpenMRNLite"
    "DCCppProtocol"
    "nlohmann_json"
    "Configuration"
)

register_component()