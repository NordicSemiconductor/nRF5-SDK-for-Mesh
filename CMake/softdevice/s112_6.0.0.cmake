set(name "s112_6.0.0")
set(dir ${SDK_ROOT}/components/softdevice)
set(${name}_INCLUDE_DIRS
    "${dir}/s112/headers/"
    "${dir}/s112/headers/nrf52/")
set(${name}_HEX_FILE
    "${dir}/s112/hex/s112_nrf52_6.0.0_softdevice.hex")
set(${name}_VERSION "6.0.0")
set(${name}_MAJOR_VERSION "6")
set(${name}_DEFINES
    -DS112
    -DSOFTDEVICE_PRESENT
    -DNRF_SD_BLE_API_VERSION=${${name}_MAJOR_VERSION})
