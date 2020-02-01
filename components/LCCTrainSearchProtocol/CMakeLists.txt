set(COMPONENT_SRCDIRS
    "src"
)

set(COMPONENT_ADD_INCLUDEDIRS
    "src"
)

set(COMPONENT_REQUIRES
    "OpenMRNLite"
)

register_component()

set_source_files_properties(src/FindProtocolDefs.cpp PROPERTIES COMPILE_FLAGS -Wno-type-limits)