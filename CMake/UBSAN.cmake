if (CMAKE_HOST_UNIX)
    option(ENABLE_UBSAN "Enable the Undefined Behavior Sanitizer" ON)

    if (ENABLE_UBSAN)
        include(CheckCCompilerFlag)
        message(STATUS "Undefined behaviour sanitizer enabled")

        set(CMAKE_REQUIRED_LIBRARIES -lubsan -lasan) # Libraries required for the checking below

        set(UBSAN_COMPILER_FLAGS "-fno-sanitize-recover")
        mark_as_advanced(UBSAN_COMPILER_FLAGS)

        check_c_compiler_flag("-fsanitize=bounds" UBSAN_SANITIZE_BOUNDS_SUPPORTED)
        if (UBSAN_SANITIZE_BOUNDS_SUPPORTED)
            set(UBSAN_COMPILER_FLAGS "${UBSAN_COMPILER_FLAGS} -fsanitize=bounds")
            set(UBSAN_FEATURES_ENABLED "${UBSAN_FEATURES_ENABLED} +bounds")
        endif ()

        check_c_compiler_flag("-fsanitize=address" UBSAN_SANITIZE_ADDRESS_SUPPORTED)
        if (UBSAN_SANITIZE_ADDRESS_SUPPORTED)
            set(UBSAN_COMPILER_FLAGS "${UBSAN_COMPILER_FLAGS} -fsanitize=address")
            set(UBSAN_FEATURES_ENABLED "${UBSAN_FEATURES_ENABLED} +address")
        endif ()

        check_c_compiler_flag("-fsanitize=undefined" UBSAN_SANITIZE_UNDEFINED_SUPPORTED)
        if (UBSAN_SANITIZE_UNDEFINED_SUPPORTED)
            set(UBSAN_COMPILER_FLAGS "${UBSAN_COMPILER_FLAGS} -fsanitize=undefined")
            set(UBSAN_FEATURES_ENABLED "${UBSAN_FEATURES_ENABLED} +undefined")
        endif ()

        message(STATUS "UBSAN/ASAN features:${UBSAN_FEATURES_ENABLED}")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${UBSAN_COMPILER_FLAGS}")
        set(CMAKE_LD_FLAGS "${CMAKE_LD_FLAGS} ${UBSAN_COMPILER_FLAGS}")
    endif (ENABLE_UBSAN)
endif (CMAKE_HOST_UNIX)

