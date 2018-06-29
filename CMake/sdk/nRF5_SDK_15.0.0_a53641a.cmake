set(nRF5_SDK_15.0.0_a53641a_SOURCE_FILES
    "${SDK_ROOT}/components/libraries/util/app_error.c"
    "${SDK_ROOT}/components/libraries/util/app_util_platform.c"
    "${SDK_ROOT}/components/libraries/timer/app_timer.c")

set(nRF5_SDK_15.0.0_a53641a_INCLUDE_DIRS
    "${SDK_ROOT}/integration/nrfx"
    "${SDK_ROOT}/components/libraries/util"
    "${SDK_ROOT}/components/libraries/timer"
    "${SDK_ROOT}/components/libraries/experimental_section_vars"
    "${SDK_ROOT}/components/libraries/delay")

if (TOOLCHAIN STREQUAL "armcc")
    set(nRF5_SDK_15.0.0_a53641a_SOURCE_FILES
        ${nRF5_SDK_15.0.0_a53641a_SOURCE_FILES}
        "${SDK_ROOT}/components/libraries/util/app_error_handler_keil.c")
else ()
    set(nRF5_SDK_15.0.0_a53641a_SOURCE_FILES
        ${nRF5_SDK_15.0.0_a53641a_SOURCE_FILES}
        "${SDK_ROOT}/components/libraries/util/app_error_handler_gcc.c")
endif ()
