set(src_dirs
    "src/dcc"
    "src/executor"
    "src/freertos_drivers/esp32"
    "src/freertos_drivers/arduino"
    "src/openlcb"
    "src/os"
    "src/utils"
)

if(CONFIG_ENABLE_ARDUINO_DEPENDS)
    list(APPEND src_dirs "src")
endif()

set(COMPONENT_SRCDIRS "${src_dirs}" )

set(COMPONENT_ADD_INCLUDEDIRS "src" )

set(COMPONENT_REQUIRES "mdns" )

register_component()

target_compile_options(${COMPONENT_LIB} PUBLIC -DESP32 -DARDUINO)

set_source_files_properties(src/utils/FileUtils.cpp PROPERTIES COMPILE_FLAGS -Wno-type-limits)
set_source_files_properties(src/openlcb/SimpleStack.cpp PROPERTIES COMPILE_FLAGS -Wno-implicit-fallthrough)