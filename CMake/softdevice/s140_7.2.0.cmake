set(name "s140_7.2.0")
set(dir ${SDK_ROOT}/components/softdevice)
set(${name}_INCLUDE_DIRS
    "${dir}/s140/headers/"
    "${dir}/s140/headers/nrf52/")
set(${name}_HEX_FILE
    "${dir}/s140/hex/s140_nrf52_7.2.0_softdevice.hex")
set(${name}_VERSION "7.2.0")
set(${name}_MAJOR_VERSION "7")
set(${name}_DEFINES
    -DS140
    -DSOFTDEVICE_PRESENT
    -DNRF_SD_BLE_API_VERSION=${${name}_MAJOR_VERSION})
