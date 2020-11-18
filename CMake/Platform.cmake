if (BUILD_HOST)
    set(PLATFORM "host" CACHE STRING "" FORCE)
else ()
    # Configure for the available platforms (nrf5*)
    set(PLATFORM "nrf52832_xxAA"
        CACHE STRING "Choose the target platform to build for. Use \"host\" for unit test builds.")
    set_property(CACHE PLATFORM PROPERTY STRINGS
        "nrf52832_xxAA" "nrf52820_xxAA" "nrf52833_xxAA" "nrf52840_xxAA" "nrf52810_xxAA" "nrf51422_xxAC")
endif ()

if (NOT EXISTS "${CMAKE_CONFIG_DIR}/platform/${PLATFORM}.cmake")
    get_property(SUPPORTED CACHE PLATFORM PROPERTY STRINGS)
    message(FATAL_ERROR "Platform specific file for ${PLATFORM} not found. Supported options: ${SUPPORTED}")
endif()
