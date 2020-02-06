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

###############################################################################
# Switch from GNU++11 to C++14
###############################################################################

string(REPLACE "-std=gnu++11" "-std=c++14" CXX_OPTIONS "${CXX_COMPILE_OPTIONS}" )
idf_build_set_property(CXX_COMPILE_OPTIONS "${CXX_OPTIONS}" REPLACE)

###############################################################################
# Add required compilation flags for customization of OpenMRNLite
###############################################################################

idf_build_set_property(COMPILE_DEFINITIONS "-DESP32_WIFIMGR_SOCKETPARAMS_LOG_LEVEL=VERBOSE" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "-DESP32_WIFIMGR_MDNS_LOOKUP_LOG_LEVEL=VERBOSE" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "-DLOCKED_LOGGING" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "-DOPENMRN_EXCLUDE_REBOOT_IMPL" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "-DOPENMRN_EXCLUDE_FREE_HEAP_IMPL" APPEND)

###############################################################################
# Enable usage of std::stoi/stol/etc
###############################################################################

idf_build_set_property(COMPILE_DEFINITIONS "-D_GLIBCXX_USE_C99" APPEND)

###############################################################################
# Search for GZIP application
###############################################################################

FIND_PROGRAM(GZIP
  NAMES gzip
  PATHS /bin
        /usr/bin
        /usr/local/bin
)

if(NOT GZIP)
  message(FATAL_ERROR "Unable to find 'gzip' program")
endif()

###############################################################################
# Generate a compressed version of web content on-demand
###############################################################################

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

###############################################################################
# Add web content to the binary
###############################################################################

target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "${CMAKE_CURRENT_SOURCE_DIR}/data/index.html.gz" BINARY)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "${CMAKE_CURRENT_SOURCE_DIR}/data/jqClock-lite.min.js.gz" BINARY)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.min.js.gz" BINARY)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.mobile-1.5.0-rc1.min.js.gz" BINARY)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.mobile-1.5.0-rc1.min.css.gz" BINARY)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "${CMAKE_CURRENT_SOURCE_DIR}/data/jquery.simple.websocket.min.js.gz" BINARY)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "${CMAKE_CURRENT_SOURCE_DIR}/data/ajax-loader.gif" BINARY)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "${CMAKE_CURRENT_SOURCE_DIR}/data/loco-32x32.png" BINARY)

###############################################################################
# Configuration validations
###############################################################################

if(NOT CONFIG_FREERTOS_HZ EQUAL 1000)
    message(FATAL_ERROR "FreeRTOS tick rate (hz) is required to be 1000.")
endif()

if(NOT CONFIG_PARTITION_TABLE_FILENAME STREQUAL "ESP32CS-partitions.csv")
    message(FATAL_ERROR "The custom partition table option is not enabled in menuconfig and is required for compilation.")
endif()

if(NOT CONFIG_PARTITION_TABLE_CUSTOM_FILENAME STREQUAL "ESP32CS-partitions.csv")
    message(FATAL_ERROR "The custom partition table option is not enabled in menuconfig and is required for compilation.")
endif()

if(NOT CONFIG_PARTITION_TABLE_CUSTOM)
    message(FATAL_ERROR "The custom partition table option is not enabled in menuconfig and is required for compilation.")
endif()

if(NOT CONFIG_LWIP_SO_RCVBUF)
    message(FATAL_ERROR "LwIP SO_RCVBUF is a required option in menuconfig.")
endif()

if(CONFIG_ESP_MAIN_TASK_STACK_SIZE LESS 8192)
    message(FATAL_ERROR "Main task stack size must be at least 8192 bytes.")
endif()

###############################################################################
# Ensure SSID and PASSWORD are provided.
###############################################################################
if (CONFIG_WIFI_MODE_STATION OR CONFIG_WIFI_MODE_SOFTAP_STATION)
    if (NOT CONFIG_WIFI_SSID OR NOT CONFIG_WIFI_PASSWORD)
        message(FATAL_ERROR "WiFi SSID & Password are required when WiFi mode is STATION or STATION+SoftAP.")
    endif()
endif()

###############################################################################
# LCC interface configuration validations
###############################################################################
if (CONFIG_LCC_NODE_ID EQUAL "")
    message(FATAL_ERROR "LCC Node ID must be defined")
endif()

###############################################################################
# OPS H-Bridge validations
###############################################################################

if (NOT CONFIG_OPS_ENABLE_PIN OR NOT CONFIG_OPS_SIGNAL_PIN OR NOT CONFIG_OPS_PREAMBLE_BITS OR CONFIG_OPS_ADC STREQUAL "")
    message(FATAL_ERROR "One (or more) OPS H-Bridge required parameters is not defined.")
endif()

if (CONFIG_OPS_PREAMBLE_BITS LESS 11 AND NOT CONFIG_OPS_RAILCOM)
    message(FATAL_ERROR "OPS track preamble bits is too low, at least 11 bits are required.")
endif()

if (CONFIG_OPS_PREAMBLE_BITS LESS 16 AND CONFIG_OPS_RAILCOM)
    message(FATAL_ERROR "OPS track preamble bits is too low, at least 16 bits are required when RailCom is enabled.")
endif()

if (CONFIG_OPS_PREAMBLE_BITS GREATER 20)
    message(FATAL_ERROR "OPS track preamble bits is too high, a maximum of 20 bits is supported.")
endif()

if (CONFIG_OPS_RAILCOM AND (CONFIG_OPS_ENABLE_PIN EQUAL CONFIG_OPS_RAILCOM_ENABLE_PIN))
    message(FATAL_ERROR "OPS RailCom enable pin and OPS H-Bridge enable pin must be unique.")
endif()

if (CONFIG_OPS_RAILCOM AND (CONFIG_OPS_SIGNAL_PIN EQUAL CONFIG_OPS_RAILCOM_ENABLE_PIN))
    message(FATAL_ERROR "OPS RailCom enable pin and OPS H-Bridge signal pin must be unique.")
endif()

if (CONFIG_OPS_RAILCOM AND (CONFIG_OPS_ENABLE_PIN EQUAL CONFIG_OPS_RAILCOM_BRAKE_PIN))
    message(FATAL_ERROR "OPS RailCom brake pin and OPS H-Bridge enable pin must be unique.")
endif()

if (CONFIG_OPS_RAILCOM AND (CONFIG_OPS_SIGNAL_PIN EQUAL CONFIG_OPS_RAILCOM_BRAKE_PIN))
    message(FATAL_ERROR "OPS RailCom brake pin and OPS H-Bridge signal pin must be unique.")
endif()

if (CONFIG_OPS_RAILCOM AND (CONFIG_OPS_ENABLE_PIN EQUAL CONFIG_OPS_RAILCOM_SHORT_PIN))
    message(FATAL_ERROR "OPS RailCom short pin and OPS H-Bridge enable pin must be unique.")
endif()

if (CONFIG_OPS_RAILCOM AND (CONFIG_OPS_SIGNAL_PIN EQUAL CONFIG_OPS_RAILCOM_SHORT_PIN))
    message(FATAL_ERROR "OPS RailCom short pin and OPS H-Bridge signal pin must be unique.")
endif()

if (CONFIG_OPS_RAILCOM AND (CONFIG_OPS_ENABLE_PIN EQUAL CONFIG_OPS_RAILCOM_UART_RX_PIN))
    message(FATAL_ERROR "OPS RailCom UART RX pin and OPS H-Bridge enable pin must be unique.")
endif()

if (CONFIG_OPS_RAILCOM AND (CONFIG_OPS_SIGNAL_PIN EQUAL CONFIG_OPS_RAILCOM_UART_RX_PIN))
    message(FATAL_ERROR "OPS RailCom UART RX pin and OPS H-Bridge signal pin must be unique.")
endif()

###############################################################################
# PROG H-Bridge validations
###############################################################################

if (NOT CONFIG_PROG_ENABLE_PIN OR NOT CONFIG_PROG_SIGNAL_PIN OR NOT CONFIG_PROG_PREAMBLE_BITS OR CONFIG_PROG_ADC STREQUAL "")
    message(FATAL_ERROR "One (or more) PROG H-Bridge required parameters is not defined.")
endif()

if (CONFIG_PROG_PREAMBLE_BITS LESS 22)
    message(FATAL_ERROR "PROG track preamble bits is too low, at least 22 bits are required.")
endif()

if (CONFIG_PROG_PREAMBLE_BITS GREATER 75)
    message(FATAL_ERROR "PROG track preamble bits is too high, a maximum of 75 bits is supported.")
endif()

if (CONFIG_OPS_ENABLE_PIN EQUAL CONFIG_PROG_ENABLE_PIN)
    message(FATAL_ERROR "OPS and PROG H-Bridge enable pin must be unique.")
endif()

if (CONFIG_OPS_SIGNAL_PIN EQUAL CONFIG_PROG_SIGNAL_PIN)
    message(FATAL_ERROR "OPS and PROG H-Bridge signal pin must be unique.")
endif()

if (CONFIG_STATUS_LED AND (CONFIG_STATUS_LED_DATA_PIN EQUAL CONFIG_OPS_ENABLE_PIN))
    message(FATAL_ERROR "Status LED data pin and OPS H-Bridge enable pin must be unique.")
endif()

if (CONFIG_STATUS_LED AND (CONFIG_STATUS_LED_DATA_PIN EQUAL CONFIG_PROG_ENABLE_PIN))
    message(FATAL_ERROR "Status LED data pin and PROG H-Bridge enable pin must be unique.")
endif()

if (CONFIG_STATUS_LED AND (CONFIG_STATUS_LED_DATA_PIN EQUAL CONFIG_OPS_SIGNAL_PIN))
    message(FATAL_ERROR "Status LED data pin and OPS H-Bridge signal pin must be unique.")
endif()

if (CONFIG_STATUS_LED AND (CONFIG_STATUS_LED_DATA_PIN EQUAL CONFIG_PROG_SIGNAL_PIN))
    message(FATAL_ERROR "Status LED data pin and PROG H-Bridge signal pin must be unique.")
endif()