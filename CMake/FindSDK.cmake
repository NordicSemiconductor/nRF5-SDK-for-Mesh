set(nRF5_SDK_VERSION "nRF5_SDK_17.0.2_d674dde" CACHE STRING "nRF5 SDK")

if (NOT nRF5_SDK_VERSION)
    message(FATAL_ERROR "You need to specifiy a nRF5_SDK_VERSION to use.")
endif()

find_program(PATCH_EXECUTABLE patch
    DOC "Path to `patch` command line executable")

set(PATCH_COMMAND "")
if (PATCH_EXECUTABLE)
    if (CMAKE_HOST_WIN32)
        set(PATCH_FILE "${CMAKE_CONFIG_DIR}/sdk/${nRF5_SDK_VERSION}.patch.win32")
    else ()
        set(PATCH_FILE "${CMAKE_CONFIG_DIR}/sdk/${nRF5_SDK_VERSION}.patch.unix")
    endif (CMAKE_HOST_WIN32)

    if (EXISTS PATCH_FILE)
        set(PATCH_COMMAND patch -p0 -i ${PATCH_FILE})
    else ()
        set(PATCH_COMMAND "")
    endif()
else ()
    message(WARNING
        "Could not find `patch` executable. \
        Automatic patching of the nRF5 SDK not supported. \
        See ${PATCH_FILE} for diff to apply.")
endif (PATCH_EXECUTABLE)

set(DEFAULT_SDK_ROOT "${CMAKE_SOURCE_DIR}/../${nRF5_SDK_VERSION}")
find_dependency(SDK_ROOT
    "Path to nRF5 SDK root"
    "${DEFAULT_SDK_ROOT}"
    "license.txt")

if (NOT SDK_ROOT)
    include(ExternalProject)

    string(REGEX REPLACE "(nRF5)([1]?_SDK_)([0-9]*).*" "\\1\\2v\\3.x.x" SDK_DIR ${nRF5_SDK_VERSION})
    set(nRF5_SDK_URL "https://developer.nordicsemi.com/nRF5_SDK/${SDK_DIR}/${nRF5_SDK_VERSION}.zip")

    ExternalProject_Add(nRF5_SDK
        PREFIX "${nRF5_SDK_VERSION}"
        TMP_DIR "${CMAKE_CURRENT_BINARY_DIR}/${nRF5_SDK_VERSION}"
        SOURCE_DIR "${DEFAULT_SDK_ROOT}/"
        DOWNLOAD_DIR "${DEFAULT_SDK_ROOT}/zip"
        DOWNLOAD_NAME "${nRF5_SDK_VERSION}.zip"
        URL ${nRF5_SDK_URL}
        PATCH_COMMAND ${PATCH_COMMAND}
        # No build or configure commands
        CONFIGURE_COMMAND ""
        BUILD_COMMAND ""
        INSTALL_COMMAND ""
        LOG_DOWNLOAD ON
        EXCLUDE_FROM_ALL ON)
    message(WARNING "
  Could not find the nRF5 SDK. The build will fail.
  Please run the nRF5_SDK target to download it or provide the correct path
  using the -DSDK_ROOT option or setting the SDK_ROOT environment variable.
  After the download is complete, re-run `cmake`.\n")
endif ()
