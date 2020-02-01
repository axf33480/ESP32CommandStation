set(COMPONENT_SRCDIRS "src" )

set(COMPONENT_ADD_INCLUDEDIRS "src" )

set(COMPONENT_REQUIRES "OpenMRNLite" "lwip" )

register_component()
set_source_files_properties(src/HttpRequestWebSocket.cpp PROPERTIES COMPILE_FLAGS -Wno-implicit-fallthrough)
set_source_files_properties(src/HttpRequestFlow.cpp PROPERTIES COMPILE_FLAGS -Wno-implicit-fallthrough)