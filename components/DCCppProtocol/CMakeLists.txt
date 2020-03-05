
set(COMPONENT_SRCS
    "DCCppProtocol.cpp"
    "DCCProgrammer.cpp"
)

set(COMPONENT_ADD_INCLUDEDIRS "include" )

set(COMPONENT_REQUIRES
    "DCCSignalGenerator"
    "Esp32HttpServer"
    "DCCTurnoutManager"
)

register_component()