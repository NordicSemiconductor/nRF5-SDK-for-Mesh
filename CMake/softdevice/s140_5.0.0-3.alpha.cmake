set(name "s140_5.0.0-3.alpha")
set(dir ${CMAKE_SOURCE_DIR}/external/softdevice/${name})
set(${name}_INCLUDE_DIRS
    "${dir}/s140_nrf52840_5.0.0-3.alpha_API/include/"
    "${dir}/s140_nrf52840_5.0.0-3.alpha_API/include/nrf52")
set(${name}_HEX_FILE
    "${dir}/s140_nrf52840_5.0.0-3.alpha_softdevice.hex")
set(${name}_VERSION "5.0.0-3.alpha")
set(${name}_MAJOR_VERSION "5")
set(${name}_DEFINES
    -DS140
    -DSOFTDEVICE_PRESENT
    -DSD_BLE_API_VERSION=${${name}_MAJOR_VERSION}
    -DNRF_SD_BLE_API_VERSION=${${name}_MAJOR_VERSION})
set(${name}_FLASH_SIZE "147456")
set(${name}_RAM_SIZE "12728")
