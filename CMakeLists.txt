cmake_minimum_required(VERSION 3.6)

if (NOT CMAKE_VERSION VERSION_LESS 3.9)
    # Allow user to enable CMAKE_INTERPROCEDURAL_OPTIMIZATION (LTO) if supported for the toolchain.
    # This is supported from CMake version 9 and later.
    cmake_policy(SET CMP0069 NEW)
endif ()

set(VERSION_MAJOR 5)
set(VERSION_MINOR 0)
set(VERSION_BUGFIX 0)
set(VERSION_SUFFIX "")
set(VERSION_STRING "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_BUGFIX}${VERSION_SUFFIX}" CACHE STRING "" FORCE)

message(STATUS "Configuring CMake for nRF5 SDK for Bluetooth Mesh ${VERSION_STRING}")

set(NRF_MESH_TEST_BUILD 0 CACHE STRING "")

option(BUILD_HOST "Build for host (unit test build)" OFF)
option(BUILD_EXAMPLES "Build all examples with default target." ON)

option(EXPERIMENTAL_INSTABURST_ENABLED "Use experimental Instaburst feature." OFF)
set(MESH_MEM_BACKEND "stdlib" CACHE STRING "Mesh dynamic memory manager backend")

if (NOT BUILD_HOST)
    set(CMAKE_SYSTEM_NAME "Generic")
    set(CMAKE_SYSTEM_PROCESSOR "ARM")
endif (NOT BUILD_HOST)

# We enable the project() here for CMake to initialize variables s.a. "CMAKE_HOST_W32".
# Languages are enabled _after_ the toolchain has been setup.
project(MBTLE
    VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_BUGFIX}
    LANGUAGES NONE)

set(CMAKE_CONFIG_DIR "${CMAKE_SOURCE_DIR}/CMake")
set(CMAKE_SOURCE_DIR ${CMAKE_SOURCE_DIR} CACHE STRING "" FORCE)

# Needed tools for generating documentation and serial PyACI
find_package(PythonInterp)
find_package(Doxygen)
find_program(DOT_EXECUTABLE "dot" PATHS ENV PATH)
find_program(MSCGEN_EXECUTABLE "mscgen" PATHS ENV PATH)

include("${CMAKE_CONFIG_DIR}/Toolchain.cmake")
include("${CMAKE_CONFIG_DIR}/Platform.cmake")
include("${CMAKE_CONFIG_DIR}/SoftDevice.cmake")
include("${CMAKE_CONFIG_DIR}/FindDependency.cmake")
include("${CMAKE_CONFIG_DIR}/FindSDK.cmake")

# We have to return manually from here s.t. the CMake generation
# doesn't stop and we have the nRF5_SDK target available.
if (NOT SDK_ROOT)
    return()
endif ()

include("${CMAKE_CONFIG_DIR}/BuildType.cmake")
include("${CMAKE_CONFIG_DIR}/Board.cmake")
include("${CMAKE_CONFIG_DIR}/PCLint.cmake")
include("${CMAKE_CONFIG_DIR}/GenerateSESProject.cmake")

include("${CMAKE_CONFIG_DIR}/sdk/${nRF5_SDK_VERSION}.cmake")
include("${CMAKE_CONFIG_DIR}/platform/${PLATFORM}.cmake")
include("${CMAKE_CONFIG_DIR}/softdevice/${SOFTDEVICE}.cmake")
include("${CMAKE_CONFIG_DIR}/board/${BOARD}.cmake")

message(STATUS "SDK: ${nRF5_SDK_VERSION}")
message(STATUS "Platform: ${PLATFORM}")
message(STATUS "Arch: ${${PLATFORM}_ARCH}")
message(STATUS "SoftDevice: ${SOFTDEVICE}")
message(STATUS "Board: ${BOARD}")

set(ARCH ${${PLATFORM}_ARCH})

enable_language(C ASM)

if (NOT BUILD_HOST)
    set(CMAKE_EXECUTABLE_SUFFIX ".elf")
    set(BUILD_SHARED_LIBS OFF)
    set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
else ()
    message(STATUS "Building for HOST")
    include("${CMAKE_CONFIG_DIR}/UnitTest.cmake")
    include("${CMAKE_CONFIG_DIR}/Coverage.cmake")
    include("${CMAKE_CONFIG_DIR}/UBSAN.cmake")
endif ()

if (EXPERIMENTAL_INSTABURST_ENABLED)
    if (PLATFORM STREQUAL "nrf52832_xxAA")
        add_definitions("-DEXPERIMENTAL_INSTABURST_ENABLED")
    else()
        message(WARNING "Instaburst is only available on nrf52832_xxAA")
        set(EXPERIMENTAL_INSTABURST_ENABLED OFF)
    endif()
endif()

if (DEFINED PERSISTENT_STORAGE)
    add_definitions("-DPERSISTENT_STORAGE=$<BOOL:${PERSISTENT_STORAGE}>")
endif()

if (DEFINED SCENE_SETUP_SERVER_INSTANCES_MAX)
    add_definitions("-DSCENE_SETUP_SERVER_INSTANCES_MAX=${SCENE_SETUP_SERVER_INSTANCES_MAX}")
endif()

# Export compilation commands to .json file (used by clang-complete backends)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory("mesh")
add_subdirectory("external")

if (NOT PYTHON_EXECUTABLE)
    message(WARNING "Need python executable to generate serial documentation and PyACI")
elseif (NOT DOXYGEN_EXECUTABLE)
    message(WARNING "Doxygen not found, documentation build is not available")
else ()
    add_subdirectory("tools")
    add_subdirectory("doc")
endif ()

if (NOT BUILD_HOST)
    include("${CMAKE_CONFIG_DIR}/Nrfjprog.cmake")
    add_subdirectory("models")
    if (BUILD_EXAMPLES)
        add_subdirectory("examples")
    else()
        add_subdirectory("examples" EXCLUDE_FROM_ALL)
    endif ()
endif ()
