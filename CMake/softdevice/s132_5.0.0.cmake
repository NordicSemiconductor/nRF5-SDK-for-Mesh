set(name "s132_5.0.0")
set(dir "${CMAKE_SOURCE_DIR}/external/softdevice/${name}")
set(${name}_INCLUDE_DIRS
    "${dir}/s132_nrf52_5.0.0_API/include"
    "${dir}/s132_nrf52_5.0.0_API/include/nrf52")
set(${name}_HEX_FILE
    "${dir}/s132_nrf52_5.0.0_softdevice.hex")
set(${name}_VERSION "5.0.0")
set(${name}_MAJOR_VERSION "5")
set(${name}_DEFINES
    -DS132
    -DSOFTDEVICE_PRESENT
    -DNRF_SD_BLE_API_VERSION=${${name}_MAJOR_VERSION})
