set(nRF5_SDK_17.0.2_d674dde_SOURCE_FILES
    "${SDK_ROOT}/components/libraries/util/app_error.c"
    "${SDK_ROOT}/components/libraries/util/app_util_platform.c")

set(nRF5_SDK_17.0.2_d674dde_INCLUDE_DIRS
    "${SDK_ROOT}/integration/nrfx"
    "${SDK_ROOT}/components/libraries/util"
    "${SDK_ROOT}/components/libraries/timer"
    "${SDK_ROOT}/components/libraries/log"
    "${SDK_ROOT}/components/libraries/log/src"
    "${SDK_ROOT}/components/libraries/experimental_section_vars"
    "${SDK_ROOT}/components/libraries/delay"
    "${SDK_ROOT}/modules/nrfx"
    "${SDK_ROOT}/modules/nrfx/drivers/include"
    "${SDK_ROOT}/modules/nrfx/drivers"
    "${SDK_ROOT}/modules/nrfx/hal"
    "${SDK_ROOT}/modules/nrfx/mdk")

if (TOOLCHAIN STREQUAL "armcc")
    set(nRF5_SDK_17.0.2_d674dde_SOURCE_FILES
        ${nRF5_SDK_17.0.2_d674dde_SOURCE_FILES}
        "${SDK_ROOT}/components/libraries/util/app_error_handler_keil.c")
else ()
    set(nRF5_SDK_17.0.2_d674dde_SOURCE_FILES
        ${nRF5_SDK_17.0.2_d674dde_SOURCE_FILES}
        "${SDK_ROOT}/components/libraries/util/app_error_handler_gcc.c")
endif ()
