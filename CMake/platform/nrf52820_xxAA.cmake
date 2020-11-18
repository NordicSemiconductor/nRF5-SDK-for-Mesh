set(nrf52820_xxAA_ARCH "cortex-m4")
set(nrf52820_xxAA_SOURCE_FILES
    "${SDK_ROOT}/modules/nrfx/mdk/system_nrf52820.c")

set(nrf52820_xxAA_INCLUDE_DIRS
    "${SDK_ROOT}/modules/nrfx"
    "${SDK_ROOT}/modules/nrfx/mdk"
    "${SDK_ROOT}/modules/nrfx/hal"
    "${SDK_ROOT}/components/toolchain/cmsis/include")

if (TOOLCHAIN MATCHES "gcc" OR TOOLCHAIN STREQUAL "clang")
    set(nrf52820_xxAA_SOURCE_FILES
        ${nrf52820_xxAA_SOURCE_FILES}
        "${SDK_ROOT}/modules/nrfx/mdk/gcc_startup_nrf52820.S")
    set(nrf52820_xxAA_INCLUDE_DIRS
        ${nrf52820_xxAA_INCLUDE_DIRS}
        "${SDK_ROOT}/components/toolchain/gcc"
        "${SDK_ROOT}/components/toolchain/cmsis/dsp/GCC")
    set(nrf52820_xxAA_LINK_INCLUDE_DIR
        "${SDK_ROOT}/modules/nrfx/mdk")
elseif (TOOLCHAIN STREQUAL "armcc")
    set(nrf52820_xxAA_SOURCE_FILES
        ${nrf52820_xxAA_SOURCE_FILES}
        "${SDK_ROOT}/modules/nrfx/mdk/arm_startup_nrf52820.s")
    set(nrf52820_xxAA_INCLUDE_DIRS
        ${nrf52820_xxAA_INCLUDE_DIRS}
        "${SDK_ROOT}/components/toolchain/cmsis/dsp/ARM")
else ()
    message(FATAL_ERROR "Unknown toolchain ${TOOLCHAIN}")
endif ()

set(nrf52820_xxAA_DEFINES
    -DNRF52_SERIES
    -DNRF52820
    -DNRF52820_XXAA
    -DNRF_MESH_LOG_ENABLE=0)

set(nrf52820_xxAA_FAMILY "NRF52")

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(WARNING
        "The program might be too large for the nRF52820 in Debug mode. "
        "Using CMAKE_BUILD_TYPE=MinSizeRel is recommended.")
endif (CMAKE_BUILD_TYPE STREQUAL "Debug")