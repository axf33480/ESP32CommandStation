set(COMPONENT_SRCS
    "HC12Radio.cpp"
)

set(COMPONENT_ADD_INCLUDEDIRS "include" )

set(COMPONENT_REQUIRES
    "OpenMRNLite"
    "driver"
    "DCCppProtocol"
)

register_component()