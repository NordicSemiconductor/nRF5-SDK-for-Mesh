set(nrf52832_xxAA_ARCH "cortex-m4f")
set(nrf52832_xxAA_SOURCE_FILES
    "${SDK_ROOT}/components/toolchain/system_nrf52.c")

set(nrf52832_xxAA_INCLUDE_DIRS
    "${SDK_ROOT}/components/device"
    "${SDK_ROOT}/components/toolchain"
    "${SDK_ROOT}/components/toolchain/cmsis/include")

if (TOOLCHAIN MATCHES "gcc" OR TOOLCHAIN STREQUAL "clang")
    set(nrf52832_xxAA_SOURCE_FILES
        ${nrf52832_xxAA_SOURCE_FILES}
        "${SDK_ROOT}/components/toolchain/gcc/gcc_startup_nrf52.S")
    set(nrf52832_xxAA_INCLUDE_DIRS
        ${nrf52832_xxAA_INCLUDE_DIRS}
        "${SDK_ROOT}/components/toolchain/gcc"
        "${SDK_ROOT}/components/toolchain/dsp/GCC")
    set(nrf52832_xxAA_LINK_INCLUDE_DIR
        "${SDK_ROOT}/components/toolchain/gcc")
elseif (TOOLCHAIN STREQUAL "armcc")
    set(nrf52832_xxAA_SOURCE_FILES
        ${nrf52832_xxAA_SOURCE_FILES}
        "${SDK_ROOT}/components/toolchain/arm/arm_startup_nrf52.s")
    set(nrf52832_xxAA_INCLUDE_DIRS
        ${nrf52832_xxAA_INCLUDE_DIRS}
        "${SDK_ROOT}/components/toolchain/arm"
        "${SDK_ROOT}/components/toolchain/dsp/ARM")
else ()
    message(FATAL_ERROR "Unknown toolchain ${TOOLCHAIN}")
endif ()

set(nrf52832_xxAA_DEFINES
    -DNRF52
    -DNRF52_SERIES
    -DNRF52832
    -DNRF52832_XXAA)

set(nrf52832_xxAA_FAMILY "NRF52")
set(nrf52832_xxAA_FLASH_SIZE "524288")
set(nrf52832_xxAA_RAM_SIZE "65536")
