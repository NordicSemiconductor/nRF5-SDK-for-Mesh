set(name "s113_7.0.0")
set(dir "${CMAKE_SOURCE_DIR}/external/softdevice/${name}")
set(${name}_INCLUDE_DIRS
    "${dir}/s113_nrf52_7.0.0_API/include"
    "${dir}/s113_nrf52_7.0.0_API/include/nrf52")
set(${name}_HEX_FILE
    "${dir}/s113_nrf52_7.0.0_softdevice.hex")
set(${name}_VERSION "7.0.0")
set(${name}_MAJOR_VERSION "7")
set(${name}_DEFINES
    -DS113
    -DSOFTDEVICE_PRESENT
    -DNRF_SD_BLE_API_VERSION=${${name}_MAJOR_VERSION})
