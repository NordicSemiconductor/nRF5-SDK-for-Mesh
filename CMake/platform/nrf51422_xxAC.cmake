set(nrf51422_xxAC_ARCH "cortex-m0")
set(nrf51422_xxAC_SOURCE_FILES
    "${SDK_ROOT}/components/toolchain/system_nrf51.c")

set(nrf51422_xxAC_INCLUDE_DIRS
    "${SDK_ROOT}/components/device"
    "${SDK_ROOT}/components/toolchain"
    "${SDK_ROOT}/components/toolchain/cmsis/include")

if (TOOLCHAIN MATCHES "gcc" OR TOOLCHAIN STREQUAL "clang")
    set(nrf51422_xxAC_SOURCE_FILES
        ${nrf51422_xxAC_SOURCE_FILES}
        "${SDK_ROOT}/components/toolchain/gcc/gcc_startup_nrf51.S")
    set(nrf51422_xxAC_INCLUDE_DIRS
        ${nrf51422_xxAC_INCLUDE_DIRS}
        "${SDK_ROOT}/components/toolchain/gcc"
        "${SDK_ROOT}/components/toolchain/dsp/GCC")
    set(nrf51422_xxAC_LINK_INCLUDE_DIR
        "${SDK_ROOT}/components/toolchain/gcc")
elseif (TOOLCHAIN STREQUAL "armcc")
    set(nrf51422_xxAC_SOURCE_FILES
        ${nrf51422_xxAC_SOURCE_FILES}
        "${SDK_ROOT}/components/toolchain/arm/arm_startup_nrf51.s")
    set(nrf51422_xxAC_INCLUDE_DIRS
        ${nrf51422_xxAC_INCLUDE_DIRS}
        "${SDK_ROOT}/components/toolchain/arm"
        "${SDK_ROOT}/components/toolchain/dsp/ARM")
else ()
    message(FATAL_ERROR "Unknown toolchain ${TOOLCHAIN}")
endif ()

set(nrf51422_xxAC_DEFINES
    -DNRF51
    -DNRF51_SERIES
    -DNRF51422
    -DNRF51422_XXAC)

set(nrf51422_xxAC_FAMILY "NRF51")
set(nrf51422_xxAC_FLASH_SIZE "262144")
set(nrf51422_xxAC_RAM_SIZE "32768")
