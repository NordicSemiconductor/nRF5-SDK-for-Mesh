if (BUILD_HOST)
    set(SOFTDEVICE "s132_6.1.0" CACHE STRING "nRF52832 SoftDevice")
    set_property(CACHE SOFTDEVICE PROPERTY STRINGS
        "s132_6.1.0" "s132_6.0.0")
elseif (PLATFORM MATCHES "nrf51.*")
    set(SOFTDEVICE "s130_2.0.1" CACHE STRING "nRF51 SoftDevice")
    set_property(CACHE SOFTDEVICE PROPERTY STRINGS
        "s130_2.0.1")
elseif (PLATFORM MATCHES "nrf52810.*")
    set(SOFTDEVICE "s112_6.1.0" CACHE STRING "nRF52810 SoftDevice")
    set_property(CACHE SOFTDEVICE PROPERTY STRINGS
        "s112_6.1.0" "s112_6.0.0")
elseif (PLATFORM MATCHES "nrf52832.*")
    set(SOFTDEVICE "s132_6.1.0" CACHE STRING "nRF52832 SoftDevice")
    set_property(CACHE SOFTDEVICE PROPERTY STRINGS
        "s132_6.1.0" "s132_6.0.0" "s132_5.0.0")
elseif (PLATFORM MATCHES "nrf52840.*")
    set(SOFTDEVICE "s140_6.1.0" CACHE STRING "nRF52840 SoftDevice")
    set_property(CACHE SOFTDEVICE PROPERTY STRINGS
        "s140_6.1.0" "s140_6.0.0")
endif ()

# Validate SoftDevice
get_property(SOFTDEVICE_VERSIONS CACHE SOFTDEVICE PROPERTY STRINGS)
if (NOT SOFTDEVICE IN_LIST SOFTDEVICE_VERSIONS)
    list(GET SOFTDEVICE_VERSIONS 0 new_softdevice)
    message("WARNING: "
        "SoftDevice \"${SOFTDEVICE}\" not specified for platform \"${PLATFORM}\". "
        "Setting SoftDevice to ${new_softdevice}")
    # Force new version
    set(SOFTDEVICE ${new_softdevice} CACHE STRING "" FORCE)
endif ()
