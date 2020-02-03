# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(PROJECT_VER "1.5.0")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

if(IDF_VERSION_MAJOR LESS 4)
    message(FATAL_ERROR "ESP32CommandStation requires IDF v4.0 or later")
endif()

set(SUPPORTED_TARGETS esp32)
project(ESP32CommandStation)

string(REPLACE "-std=gnu++11" "-std=c++14" CXX_OPTIONS "${CXX_COMPILE_OPTIONS}" )
idf_build_set_property(CXX_COMPILE_OPTIONS "${CXX_OPTIONS}" REPLACE)

idf_build_set_property(COMPILE_DEFINITIONS "-DESP32_WIFIMGR_SOCKETPARAMS_LOG_LEVEL=VERBOSE" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "-DESP32_WIFIMGR_MDNS_LOOKUP_LOG_LEVEL=VERBOSE" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "-D_GLIBCXX_USE_C99" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "-DLOCKED_LOGGING" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "-DOPENMRN_EXCLUDE_REBOOT_IMPL" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "-DOPENMRN_EXCLUDE_FREE_HEAP_IMPL" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "-DESP32CS_CDI_VERSION=0x0150" APPEND)

FIND_PROGRAM(GZIP
  NAMES gzip
  PATHS /bin
        /usr/bin
        /usr/local/bin
)

if(NOT GZIP)
  message(FATAL_ERROR "Unable to find 'gzip' program")
endif()

add_custom_command(OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/data/index.html.gz"
    COMMAND ${GZIP} -fk ${CMAKE_CURRENT_SOURCE_DIR}/data/index.html
    VERBATIM)
set_property(TARGET ${CMAKE_PROJECT_NAME}.elf APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/data/index.html.gz")

add_custom_command(OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/data/jqClock-lite.min.js.gz"
    COMMAND ${GZIP} -fk ${CMAKE_CURRENT_SOURCE_DIR}/data/jqClock-lite.min.js
    VERBATIM)
set_property(TARGET ${CMAKE_PROJECT_NAME}.elf APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/data/jqClock-lite.min.js.gz")

add_custom_command(OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.min.js.gz"
    COMMAND ${GZIP} -fk ${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.min.js
    VERBATIM)
set_property(TARGET ${CMAKE_PROJECT_NAME}.elf APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.min.js.gz")

add_custom_command(OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.mobile-1.5.0-rc1.min.js.gz"
    COMMAND ${GZIP} -fk ${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.mobile-1.5.0-rc1.min.js
    VERBATIM)
set_property(TARGET ${CMAKE_PROJECT_NAME}.elf APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.mobile-1.5.0-rc1.min.js.gz")

add_custom_command(OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.mobile-1.5.0-rc1.min.css.gz"
    COMMAND ${GZIP} -fk ${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.mobile-1.5.0-rc1.min.css
    VERBATIM)
set_property(TARGET ${CMAKE_PROJECT_NAME}.elf APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.mobile-1.5.0-rc1.min.css.gz")

add_custom_command(OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.simple.websocket.min.js.gz"
    COMMAND ${GZIP} -fk ${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.simple.websocket.min.js
    VERBATIM)
set_property(TARGET ${CMAKE_PROJECT_NAME}.elf APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.simple.websocket.min.js.gz")

target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "${CMAKE_CURRENT_SOURCE_DIR}/data/index.html.gz" BINARY)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "${CMAKE_CURRENT_SOURCE_DIR}/data/jqClock-lite.min.js.gz" BINARY)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.min.js.gz" BINARY)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.mobile-1.5.0-rc1.min.js.gz" BINARY)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.mobile-1.5.0-rc1.min.css.gz" BINARY)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.simple.websocket.min.js.gz" BINARY)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "${CMAKE_CURRENT_SOURCE_DIR}/data/ajax-loader.gif" BINARY)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "${CMAKE_CURRENT_SOURCE_DIR}/data/loco-32x32.png" BINARY)
