set(nrf52840_xxAA_ARCH "cortex-m4f")
set(nrf52840_xxAA_SOURCE_FILES
    "${SDK_ROOT}/components/toolchain/system_nrf52840.c")

set(nrf52840_xxAA_INCLUDE_DIRS
    "${SDK_ROOT}/components/device"
    "${SDK_ROOT}/components/toolchain"
    "${SDK_ROOT}/components/toolchain/cmsis/include")

if (TOOLCHAIN MATCHES "gcc" OR TOOLCHAIN STREQUAL "clang")
    set(nrf52840_xxAA_SOURCE_FILES
        ${nrf52840_xxAA_SOURCE_FILES}
        "${SDK_ROOT}/components/toolchain/gcc/gcc_startup_nrf52840.S")
    set(nrf52840_xxAA_INCLUDE_DIRS
        ${nrf52840_xxAA_INCLUDE_DIRS}
        "${SDK_ROOT}/components/toolchain/gcc"
        "${SDK_ROOT}/components/toolchain/dsp/GCC")
    set(nrf52840_xxAA_LINK_INCLUDE_DIR
        "${SDK_ROOT}/components/toolchain/gcc")
elseif (TOOLCHAIN STREQUAL "armcc")
    set(nrf52840_xxAA_SOURCE_FILES
        ${nrf52840_xxAA_SOURCE_FILES}
        "${SDK_ROOT}/components/toolchain/arm/arm_startup_nrf52840.s")
    set(nrf52840_xxAA_INCLUDE_DIRS
        ${nrf52840_xxAA_INCLUDE_DIRS}
        "${SDK_ROOT}/components/toolchain/arm"
        "${SDK_ROOT}/components/toolchain/dsp/ARM")
else ()
    message(FATAL_ERROR "Unknown toolchain ${TOOLCHAIN}")
endif ()

set(nrf52840_xxAA_DEFINES
    -DNRF52_SERIES
    -DNRF52840
    -DNRF52840_XXAA)

set(nrf52840_xxAA_FAMILY "NRF52")
set(nrf52840_xxAA_FLASH_SIZE "1048576")
set(nrf52840_xxAA_RAM_SIZE "262144")
