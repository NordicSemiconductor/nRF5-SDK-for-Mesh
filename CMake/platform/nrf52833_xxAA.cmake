set(nrf52833_xxAA_ARCH "cortex-m4f")
set(nrf52833_xxAA_SOURCE_FILES
    "${SDK_ROOT}/modules/nrfx/mdk/system_nrf52833.c")

set(nrf52833_xxAA_INCLUDE_DIRS
    "${SDK_ROOT}/modules/nrfx"
    "${SDK_ROOT}/modules/nrfx/mdk"
    "${SDK_ROOT}/modules/nrfx/hal"
    "${SDK_ROOT}/components/toolchain/cmsis/include")

if (TOOLCHAIN MATCHES "gcc" OR TOOLCHAIN STREQUAL "clang")
    set(nrf52833_xxAA_SOURCE_FILES
        ${nrf52833_xxAA_SOURCE_FILES}
        "${SDK_ROOT}/modules/nrfx/mdk/gcc_startup_nrf52833.S")
    set(nrf52833_xxAA_INCLUDE_DIRS
        ${nrf52833_xxAA_INCLUDE_DIRS}
        "${SDK_ROOT}/components/toolchain/gcc"
        "${SDK_ROOT}/components/toolchain/cmsis/dsp/GCC")
    set(nrf52833_xxAA_LINK_INCLUDE_DIR
        "${SDK_ROOT}/modules/nrfx/mdk")
elseif (TOOLCHAIN STREQUAL "armcc")
    set(nrf52833_xxAA_SOURCE_FILES
        ${nrf52833_xxAA_SOURCE_FILES}
        "${SDK_ROOT}/modules/nrfx/mdk/arm_startup_nrf52833.s")
    set(nrf52833_xxAA_INCLUDE_DIRS
        ${nrf52833_xxAA_INCLUDE_DIRS}
        "${SDK_ROOT}/components/toolchain/cmsis/dsp/ARM")
else ()
    message(FATAL_ERROR "Unknown toolchain ${TOOLCHAIN}")
endif ()

set(nrf52833_xxAA_DEFINES
    -DNRF52_SERIES
    -DNRF52833
    -DNRF52833_XXAA)

set(nrf52833_xxAA_FAMILY "NRF52")
