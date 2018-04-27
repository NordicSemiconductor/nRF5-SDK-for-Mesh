if (BUILD_HOST)
    # Only GCC supported for host builds
    set(TOOLCHAIN "gcc" CACHE STRING "Toolchain used for host build" FORCE)
else ()
    set(TOOLCHAIN "gccarmemb" CACHE STRING "Toolchain used for compiling the target")
    set_property(CACHE TOOLCHAIN PROPERTY STRINGS "gccarmemb" "armcc" "clang")
endif ()

if (EXISTS "${CMAKE_CONFIG_DIR}/toolchain/${TOOLCHAIN}.cmake")
    include("${CMAKE_CONFIG_DIR}/toolchain/${TOOLCHAIN}.cmake")
else ()
    message(FATAL_ERROR "Toolchain \"${TOOLCHAIN}\" not recognized.")
endif ()
