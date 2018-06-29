message(WARNING "nRF51-series is no longer officially supported.")

set(nrf51422_xxAC_ARCH "cortex-m0")
set(nrf51422_xxAC_SOURCE_FILES
    "${SDK_ROOT}/modules/nrfx/mdk/system_nrf51.c")

set(nrf51422_xxAC_INCLUDE_DIRS
    "${SDK_ROOT}/modules/nrfx"
    "${SDK_ROOT}/modules/nrfx/mdk"
    "${SDK_ROOT}/modules/nrfx/hal"
    "${SDK_ROOT}/components/toolchain/cmsis/include")

if (TOOLCHAIN MATCHES "gcc" OR TOOLCHAIN STREQUAL "clang")
    set(nrf51422_xxAC_SOURCE_FILES
        ${nrf51422_xxAC_SOURCE_FILES}
        "${SDK_ROOT}/modules/nrfx/mdk/gcc_startup_nrf51.S")
    set(nrf51422_xxAC_INCLUDE_DIRS
        ${nrf51422_xxAC_INCLUDE_DIRS}
        "${SDK_ROOT}/components/toolchain/gcc")
    set(nrf51422_xxAC_LINK_INCLUDE_DIR
        "${SDK_ROOT}/modules/nrfx/mdk")
elseif (TOOLCHAIN STREQUAL "armcc")
    set(nrf51422_xxAC_SOURCE_FILES
        ${nrf51422_xxAC_SOURCE_FILES}
        "${SDK_ROOT}/modules/nrfx/mdk/arm_startup_nrf52.s")
    set(nrf51422_xxAC_INCLUDE_DIRS
        ${nrf51422_xxAC_INCLUDE_DIRS}
        "${SDK_ROOT}/components/toolchain/arm")
else ()
    message(FATAL_ERROR "Unknown toolchain ${TOOLCHAIN}")
endif ()

set(nrf51422_xxAC_DEFINES
    -DNRF51
    -DNRF51_SERIES
    -DNRF51422
    -DNRF51422_XXAC)

set(nrf51422_xxAC_FAMILY "NRF51")
