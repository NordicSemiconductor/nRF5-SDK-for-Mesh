
# default max_path length is 260.  Ninja fails if >245 chars.  This option lets cmake use tricks to shrink the path.
if( NOT CMAKE_HOST_UNIX)
  set(CMAKE_OBJECT_PATH_MAX 240)
endif()

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
    get_property(SUPPORTED CACHE TOOLCHAIN PROPERTY STRINGS)
    message(FATAL_ERROR "Toolchain \"${TOOLCHAIN}\" not recognized. Supported options: ${SUPPORTED}")
endif ()
