set(DEPENDENCIES_LOGGING_SOURCE_FILES
    "${SDK_ROOT}/components/libraries/balloc/nrf_balloc.c"
    "${SDK_ROOT}/components/libraries/memobj/nrf_memobj.c"
    "${SDK_ROOT}/components/libraries/log/src/nrf_log_frontend.c"
    "${SDK_ROOT}/components/libraries/strerror/nrf_strerror.c"
)

set(DEPENDENCIES_LOGGING_INCLUDE_DIRS
    "${SDK_ROOT}/external/fprintf"
    "${SDK_ROOT}/components/libraries/balloc"
    "${SDK_ROOT}/components/libraries/ringbuf"
    "${SDK_ROOT}/components/libraries/memobj")

set(nRF5_SDK_15.2.0_9412b96_SOURCE_FILES
    ${DEPENDENCIES_LOGGING_SOURCE_FILES}
    "${SDK_ROOT}/components/libraries/util/app_error.c"
    "${SDK_ROOT}/components/libraries/util/app_util_platform.c")

set(nRF5_SDK_15.2.0_9412b96_INCLUDE_DIRS
    ${DEPENDENCIES_LOGGING_INCLUDE_DIRS}
    "${SDK_ROOT}/integration/nrfx"
    "${SDK_ROOT}/components/libraries/util"
    "${SDK_ROOT}/components/libraries/timer"
    "${SDK_ROOT}/components/libraries/log"
    "${SDK_ROOT}/components/libraries/log/src"
    "${SDK_ROOT}/components/libraries/experimental_section_vars"
    "${SDK_ROOT}/components/libraries/delay")

if (TOOLCHAIN STREQUAL "armcc")
    set(nRF5_SDK_15.2.0_9412b96_SOURCE_FILES
        ${nRF5_SDK_15.2.0_9412b96_SOURCE_FILES}
        "${SDK_ROOT}/components/libraries/util/app_error_handler_keil.c")
else ()
    set(nRF5_SDK_15.2.0_9412b96_SOURCE_FILES
        ${nRF5_SDK_15.2.0_9412b96_SOURCE_FILES}
        "${SDK_ROOT}/components/libraries/util/app_error_handler_gcc.c")
endif ()
