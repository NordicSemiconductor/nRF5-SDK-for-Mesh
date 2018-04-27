function (extract_softdevice SOFTDEVICE_NAME)
    if (NOT EXISTS "${CMAKE_SOURCE_DIR}/external/softdevice/${SOFTDEVICE_NAME}/${SOFTDEVICE_NAME}_API/include/ble.h")
        message(STATUS "Extracting SoftDevice ${SOFTDEVICE_NAME}...")
        make_directory("${CMAKE_SOURCE_DIR}/external/softdevice/${SOFTDEVICE_NAME}")
        execute_process(
            COMMAND cmake -E tar xvzf ${CMAKE_SOURCE_DIR}/external/softdevice/zip/${SOFTDEVICE_NAME}.zip
            WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/external/softdevice/${SOFTDEVICE_NAME}"
            OUTPUT_QUIET)
    endif ()
endfunction ()
