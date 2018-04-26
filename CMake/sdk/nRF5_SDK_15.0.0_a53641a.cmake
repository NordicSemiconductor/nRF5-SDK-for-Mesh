set(nRF5_SDK_15.0.0_a53641a_SOURCE_FILES
    "${SDK_ROOT}/components/libraries/util/app_error.c")

set(nRF5_SDK_15.0.0_a53641a_INCLUDE_DIRS
    "${SDK_ROOT}/integration/nrfx"
    "${SDK_ROOT}/components/libraries/log"
    "${SDK_ROOT}/components/libraries/util"
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
