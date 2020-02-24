set(COMPONENT_SRCDIRS "src" )

set(COMPONENT_ADD_INCLUDEDIRS "include" )

set(COMPONENT_REQUIRES
    "DCCSignalGenerator"
    "EStopGenerator"
    "Esp32HttpServer"
    "DCCTurnoutManager"
)

register_component()