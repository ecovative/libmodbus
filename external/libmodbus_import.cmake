# external/libmodbus_import.cmake
# This file can be dropped into an external project to help locate libmodbus
# It should be include()ed prior to project()

if (DEFINED ENV{LIBMODBUS_PATH} AND (NOT LIBMODBUS_PATH))
    set(LIBMODBUS_PATH $ENV{LIBMODBUS_PATH})
    message("Using LIBMODBUS_PATH from environment ('${LIBMODBUS_PATH}')")
endif ()

if (DEFINED ENV{LIBMODBUS_FETCH_FROM_GIT} AND (NOT LIBMODBUS_FETCH_FROM_GIT))
    set(LIBMODBUS_FETCH_FROM_GIT $ENV{LIBMODBUS_FETCH_FROM_GIT})
    message("Using LIBMODBUS_FETCH_FROM_GIT from environment ('${LIBMODBUS_FETCH_FROM_GIT}')")
endif ()

if (DEFINED ENV{LIBMODBUS_FETCH_FROM_GIT_PATH} AND (NOT LIBMODBUS_FETCH_FROM_GIT_PATH))
    set(LIBMODBUS_FETCH_FROM_GIT_PATH $ENV{LIBMODBUS_FETCH_FROM_GIT_PATH})
    message("Using LIBMODBUS_FETCH_FROM_GIT_PATH from environment ('${LIBMODBUS_FETCH_FROM_GIT_PATH}')")
endif ()

set(LIBMODBUS_PATH "${LIBMODBUS_PATH}" CACHE PATH "Path to libmodbus")
set(LIBMODBUS_FETCH_FROM_GIT "${LIBMODBUS_FETCH_FROM_GIT}" CACHE BOOL "Set to ON to fetch libmodbus from git if not otherwise locatable")
set(LIBMODBUS_FETCH_FROM_GIT_PATH "${LIBMODBUS_FETCH_FROM_GIT_PATH}" CACHE FILEPATH "location to download libmodbus")

if (NOT LIBMODBUS_PATH)
    if (LIBMODBUS_FETCH_FROM_GIT)
        include(FetchContent)
        set(FETCHCONTENT_BASE_DIR_SAVE ${FETCHCONTENT_BASE_DIR})
        if (LIBMODBUS_FETCH_FROM_GIT_PATH)
            get_filename_component(FETCHCONTENT_BASE_DIR "${LIBMODBUS_FETCH_FROM_GIT_PATH}" REALPATH BASE_DIR "${CMAKE_SOURCE_DIR}")
        endif ()
        FetchContent_Declare(
                libmodbus
                GIT_REPOSITORY https://github.com/yourusername/libmodbus
                GIT_TAG master
        )
        if (NOT libmodbus)
            message("Downloading libmodbus")
            FetchContent_Populate(libmodbus)
            set(LIBMODBUS_PATH ${libmodbus_SOURCE_DIR})
        endif ()
        set(FETCHCONTENT_BASE_DIR ${FETCHCONTENT_BASE_DIR_SAVE})
    else ()
        # Try sibling directory
        if (PICO_SDK_PATH AND EXISTS "${PICO_SDK_PATH}/../libmodbus")
            set(LIBMODBUS_PATH ${PICO_SDK_PATH}/../libmodbus)
            message("Defaulting LIBMODBUS_PATH as sibling of PICO_SDK_PATH: ${LIBMODBUS_PATH}")
        endif()
    endif ()
endif ()

if (NOT LIBMODBUS_PATH)
    message(FATAL_ERROR "libmodbus location was not specified. Please set LIBMODBUS_PATH.")
endif ()

set(LIBMODBUS_PATH "${LIBMODBUS_PATH}" CACHE PATH "Path to libmodbus" FORCE)

get_filename_component(LIBMODBUS_PATH "${LIBMODBUS_PATH}" REALPATH BASE_DIR "${CMAKE_BINARY_DIR}")
if (NOT EXISTS ${LIBMODBUS_PATH})
    message(FATAL_ERROR "Directory '${LIBMODBUS_PATH}' not found")
endif ()

set(LIBMODBUS_PATH ${LIBMODBUS_PATH} CACHE PATH "Path to libmodbus" FORCE)

add_subdirectory(${LIBMODBUS_PATH} libmodbus)