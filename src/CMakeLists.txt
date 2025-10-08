# src/CMakeLists.txt - Library build configuration

# =============================================================================
# Source Files
# =============================================================================

set(MODBUS_SOURCES
    modbus.c
    modbus-data.c
    modbus-rtu.c
)

# Optional TCP support
if(NOT MODBUS_RTU_ONLY)
    list(APPEND MODBUS_SOURCES modbus-tcp.c)
endif()

# =============================================================================
# Create Library Target
# =============================================================================

add_library(modbus INTERFACE)

add_library(modbus_impl STATIC
    ${MODBUS_SOURCES}
)

# Alias for consistency with naming conventions
add_library(libmodbus::modbus ALIAS modbus)

# =============================================================================
# Include Directories
# =============================================================================

target_include_directories(modbus_impl PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/pico
)

target_include_directories(modbus INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/pico
)

# =============================================================================
# Compile Definitions
# =============================================================================

target_compile_definitions(modbus_impl PUBLIC
    PICO_SDK=1
    LIBMODBUS_VERSION_MAJOR=${LIBMODBUS_VERSION_MAJOR}
    LIBMODBUS_VERSION_MINOR=${LIBMODBUS_VERSION_MINOR}
    LIBMODBUS_VERSION_MICRO=${LIBMODBUS_VERSION_MICRO}
    $<$<BOOL:${MODBUS_USE_DMA}>:MODBUS_USE_DMA>
    $<$<BOOL:${MODBUS_MULTICORE_SUPPORT}>:MODBUS_MULTICORE_SUPPORT>
)

# =============================================================================
# Link Libraries
# =============================================================================

target_link_libraries(modbus_impl PUBLIC
    pico_stdlib
    hardware_uart
    hardware_gpio
    hardware_timer
)

# Optional DMA support
if(MODBUS_USE_DMA)
    target_link_libraries(modbus_impl PUBLIC
        hardware_dma
        hardware_irq
    )
    message(STATUS "libmodbus: DMA support enabled")
endif()

# Optional multi-core support
if(MODBUS_MULTICORE_SUPPORT)
    target_link_libraries(modbus_impl PUBLIC
        pico_multicore
        pico_sync
    )
    message(STATUS "libmodbus: Multi-core support enabled")
endif()

# =============================================================================
# Compiler Options
# =============================================================================

target_compile_options(modbus_impl PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter
)

# =============================================================================
# Interface Library Links Implementation
# =============================================================================

target_link_libraries(modbus INTERFACE modbus_impl)

# =============================================================================
# Status Messages
# =============================================================================

message(STATUS "libmodbus configuration:")
message(STATUS "  Version: ${LIBMODBUS_VERSION_STRING}")
message(STATUS "  DMA: ${MODBUS_USE_DMA}")
message(STATUS "  Multi-core: ${MODBUS_MULTICORE_SUPPORT}")
message(STATUS "  RTU only: ${MODBUS_RTU_ONLY}")