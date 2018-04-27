set(name "s110_8.0.0")
set(dir ${CMAKE_SOURCE_DIR}/external/softdevice/${name})
set(${name}_INCLUDE_DIRS
    "${dir}/s110_nrf51_8.0.0_API/include/"
    "${dir}/s110_nrf51_8.0.0_API/include/nrf51")
set(${name}_HEX_FILE
    "${dir}/s110_nrf51_8.0.0_softdevice.hex")
set(${name}_VERSION "8.0.0")
set(${name}_MAJOR_VERSION "8")
set(${name}_DEFINES
    -DS110
    -DSOFTDEVICE_PRESENT)
set(${name}_FLASH_SIZE "98304")
set(${name}_RAM_SIZE "8192")
