set(name "s113_7.2.0")
set(dir ${SDK_ROOT}/components/softdevice)
set(${name}_INCLUDE_DIRS
    "${dir}/s113/headers/"
    "${dir}/s113/headers/nrf52/")
set(${name}_HEX_FILE
    "${dir}/s113/hex/s113_nrf52_7.2.0_softdevice.hex")
set(${name}_VERSION "7.2.0")
set(${name}_MAJOR_VERSION "7")
set(${name}_DEFINES
    -DS113
    -DSOFTDEVICE_PRESENT
    -DNRF_SD_BLE_API_VERSION=${${name}_MAJOR_VERSION})
