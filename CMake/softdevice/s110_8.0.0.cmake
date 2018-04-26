message(FATAL_ERROR
    "The S110 SoftDevice will not build in this version of the nRF5 SDK for mesh \
and is not supported. This file is included for reference.")

set(name "s110_8.0.0")
set(dir "${CMAKE_SOURCE_DIR}/external/softdevice/${name}")
set(${name}_INCLUDE_DIRS
    "${dir}/s110_nrf51_8.0.0_API/include"
    "${dir}/s110_nrf51_8.0.0_API/include/nrf51/")
set(${name}_HEX_FILE
    "${dir}/s110_nrf51_8.0.0_softdevice.hex")
set(${name}_VERSION "8.0.0")
set(${name}_MAJOR_VERSION "1")
set(${name}_DEFINES
    -DS110
    -DSOFTDEVICE_PRESENT
    -DNRF_SD_BLE_API_VERSION=${${name}_MAJOR_VERSION})
