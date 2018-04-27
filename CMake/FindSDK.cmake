# Finds the nRF5 SDK

set(nRF5_SDK_VERSION "nRF5_SDK_14.2.0_17b948a" CACHE STRING "")

find_dependency(SDK_ROOT
    "Path to nRF5 SDK root"
    "${CMAKE_SOURCE_DIR}/external/${nRF5_SDK_VERSION}"
    "components/toolchain/system_nrf52.c")

if (NOT SDK_ROOT)
    ExternalProject_Add(nRF5_SDK
        PREFIX "${nRF5_SDK_VERSION}"
        TMP_DIR "${CMAKE_CURRENT_BINARY_DIR}/${nRF5_SDK_VERSION}"
        SOURCE_DIR "${CMAKE_SOURCE_DIR}/external/${nRF5_SDK_VERSION}/"
        DOWNLOAD_DIR "${CMAKE_SOURCE_DIR}/external/${nRF5_SDK_VERSION}/zip"
        DOWNLOAD_NAME "${nRF5_SDK_VERSION}.zip"
        URL "https://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v14.x.x/nRF5_SDK_14.2.0_17b948a.zip"
        # No build or configure commands
        CONFIGURE_COMMAND ""
        BUILD_COMMAND ""
        INSTALL_COMMAND ""
        LOG_DOWNLOAD ON
        EXCLUDE_FROM_ALL ON)
    message(WARNING "Could not find the nRF5 SDK. The build will fail.\n"
        "Please run the nRF5_SDK target to download it or provide the correct path using the "
        "-DSDK_ROOT option or setting the SDK_ROOT environment variable.")
endif ()
