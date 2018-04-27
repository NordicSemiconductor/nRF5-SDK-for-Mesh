if (NOT PYTHON_EXECUTABLE)
    set(GENERATE_SES_PROJECTS OFF)
    message(STATUS "Segger Embedded Studio Project generation not supported (Python not found)")
else ()
    option(GENERATE_SES_PROJECTS ON "Generate Segger Embedded Studio projects?")
endif (NOT PYTHON_EXECUTABLE)

set(nrf51422_xxAC_VECTORS_FILE "${CMAKE_SOURCE_DIR}/external/nRF5_SDK_14.2.0_17b948a/components/toolchain/ses/ses_nrf51_Vectors.s")
set(nrf52832_xxAA_VECTORS_FILE "${CMAKE_SOURCE_DIR}/external/nRF5_SDK_14.2.0_17b948a/components/toolchain/ses/ses_nrf52_Vectors.s")
set(nrf52840_xxAA_VECTORS_FILE "${CMAKE_SOURCE_DIR}/external/nRF5_SDK_14.2.0_17b948a/components/toolchain/ses/ses_nrf52840_Vectors.s")
set(nRF_STARTUP_FILE "${CMAKE_SOURCE_DIR}/external/nRF5_SDK_14.2.0_17b948a/components/toolchain/ses/ses_nRF_Startup.s")

function (add_ses_project TARGET_NAME)
    if (GENERATE_SES_PROJECTS)
        get_property(target_sources TARGET ${TARGET_NAME} PROPERTY SOURCES)
        get_property(target_libs TARGET ${TARGET_NAME} PROPERTY LINK_LIBRARIES)
        get_property(target_include_dirs TARGET ${TARGET_NAME} PROPERTY INCLUDE_DIRECTORIES)
        get_property(target_defines TARGET ${TARGET_NAME} PROPERTY COMPILE_DEFINITIONS)

        foreach (lib IN ITEMS ${target_libs})
            get_property(lib_sources TARGET ${lib} PROPERTY SOURCES)
            get_property(lib_include_dirs TARGET ${lib} PROPERTY INCLUDE_DIRECTORIES)
            set(target_sources ${target_sources} ${lib_sources})
            set(target_include_dirs ${target_include_dirs} ${lib_include_dirs})
        endforeach ()
        # We'll remove the GCC one in python
        set(target_sources ${target_sources} ${nRF_STARTUP_FILE})
        set(target_sources ${target_sources} ${${PLATFORM}_VECTORS_FILE})
        set(target_defines NO_VTOR_CONFIG ${target_defines})

        list(REMOVE_DUPLICATES target_include_dirs)
        file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.json
            "{
    \"target\": {
        \"name\": \"${TARGET_NAME}\",
        \"sources\": \"${target_sources}\",
        \"includes\": \"${target_include_dirs}\",
        \"defines\":\"${target_defines}\"
    },
    \"platform\": {
        \"name\": \"${PLATFORM}\",
        \"arch\": \"${${PLATFORM}_ARCH}\",
        \"flash_size\": ${${PLATFORM}_FLASH_SIZE},
        \"ram_size\": ${${PLATFORM}_RAM_SIZE}
    },
    \"softdevice\": {
        \"hex_file\": \"${${SOFTDEVICE}_HEX_FILE}\",
        \"flash_size\": ${${SOFTDEVICE}_FLASH_SIZE},
        \"ram_size\": ${${SOFTDEVICE}_RAM_SIZE}
    }
}")

        execute_process(
            COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CONFIG_DIR}/SES/SESGenerator.py "${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.json" "${CMAKE_CURRENT_SOURCE_DIR}"
            WORKING_DIRECTORY ${CMAKE_CONFIG_DIR}/SES
            )
    endif (GENERATE_SES_PROJECTS)
endfunction ()
