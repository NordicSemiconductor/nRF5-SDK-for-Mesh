set(name "s132_3.1.0")
set(dir ${CMAKE_SOURCE_DIR}/external/softdevice/${name})
set(${name}_INCLUDE_DIRS
    "${dir}/s132_nrf52_3.1.0_API/include/"
    "${dir}/s132_nrf52_3.1.0_API/include/nrf52")
set(${name}_HEX_FILE
    "${dir}/s132_nrf52_3.1.0_softdevice.hex")
set(${name}_VERSION "3.1.0")
set(${name}_MAJOR_VERSION "3")
set(${name}_DEFINES
    -DS132
    -DSOFTDEVICE_PRESENT
    -DSD_BLE_API_VERSION=${${name}_MAJOR_VERSION}
    -DNRF_SD_BLE_API_VERSION=${${name}_MAJOR_VERSION})
set(${name}_FLASH_SIZE "126976")
set(${name}_RAM_SIZE "6592")
