if (PLATFORM STREQUAL "nrf51422_xxAC")
    set(BOARD "pca10031" CACHE STRING "Board to build examples for.")
    set_property(CACHE BOARD PROPERTY STRINGS "pca10028" "pca10031")
elseif (PLATFORM STREQUAL "nrf52832_xxAA")
    set(BOARD "pca10040" CACHE STRING "Board to build examples for.")
    set_property(CACHE BOARD PROPERTY STRINGS "pca10040")
elseif (PLATFORM STREQUAL "nrf52840_xxAA")
    set(BOARD "pca10056" CACHE STRING "Board to build examples for.")
    set_property(CACHE BOARD PROPERTY STRINGS "pca10056")
elseif (PLATFORM STREQUAL "host")
    set(BOARD "pca10040" CACHE STRING "Board to build examples for.")
    set_property(CACHE BOARD PROPERTY STRINGS "pca10040")
else()
    message(FATAL_ERROR "Board for platform ${PLATFORM} not defined.")
endif ()

if (NOT EXISTS "${CMAKE_CONFIG_DIR}/board/${BOARD}.cmake")
    message(FATAL_ERROR "Board not defined.")
endif ()

get_property(BOARD_VERSIONS CACHE BOARD PROPERTY STRINGS)
if (NOT BOARD IN_LIST BOARD_VERSIONS)
    list(GET BOARD_VERSIONS 0 new_board)
    message("WARNING: "
        "Board \"${BOARD}\" not specified for platform \"${PLATFORM}\". "
        "Setting Board to ${new_board}")
    # Force new version
    set(BOARD ${new_board} CACHE STRING "" FORCE)
endif ()
