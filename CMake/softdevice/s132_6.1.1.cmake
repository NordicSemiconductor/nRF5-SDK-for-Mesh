set(name "s132_6.1.1")
set(dir ${SDK_ROOT}/components/softdevice)
set(${name}_INCLUDE_DIRS
    "${dir}/s132/headers/"
    "${dir}/s132/headers/nrf52/")
set(${name}_HEX_FILE
    "${dir}/s132/hex/s132_nrf52_6.1.1_softdevice.hex")
set(${name}_VERSION "6.1.1")
set(${name}_MAJOR_VERSION "6")
set(${name}_DEFINES
    -DS132
    -DSOFTDEVICE_PRESENT
    -DNRF_SD_BLE_API_VERSION=${${name}_MAJOR_VERSION})
