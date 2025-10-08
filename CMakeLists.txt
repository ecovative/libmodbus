# CMakeLists.txt - Root configuration for libmodbus with Pico SDK

cmake_minimum_required(VERSION 3.13)

# Must be included before project()
if (NOT TARGET pico_sdk_init)
    message(FATAL_ERROR "Raspberry Pi Pico SDK must be imported before libmodbus")
endif()

# Version information
set(LIBMODBUS_VERSION_MAJOR 3)
set(LIBMODBUS_VERSION_MINOR 1)
set(LIBMODBUS_VERSION_MICRO 10)
set(LIBMODBUS_VERSION_STRING "${LIBMODBUS_VERSION_MAJOR}.${LIBMODBUS_VERSION_MINOR}.${LIBMODBUS_VERSION_MICRO}-pico")

project(libmodbus VERSION ${LIBMODBUS_VERSION_STRING} LANGUAGES C CXX ASM)

# Don't build if we're not being built for Pico
if (NOT PICO_PLATFORM)
    message(WARNING "libmodbus: Not building for Pico platform. Use autotools for POSIX/Windows builds.")
    return()
endif()

message(STATUS "libmodbus ${LIBMODBUS_VERSION_STRING} for Pico SDK")

# Options
option(MODBUS_USE_DMA "Enable DMA support for UART" OFF)
option(MODBUS_MULTICORE_SUPPORT "Enable multi-core safety" OFF)
option(MODBUS_RTU_ONLY "Build RTU only (no TCP)" ON)
option(MODBUS_BUILD_EXAMPLES "Build example programs" OFF)

# Add src directory
add_subdirectory(src)
