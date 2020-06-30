find_program(NRFJPROG
    nrfjprog)

find_program(MERGEHEX
    mergehex)

if (NRFJPROG AND MERGEHEX AND PYTHON_EXECUTABLE)
    add_custom_target(merge)
    function(add_flash_target target)
        # Both the manual <merge> and <flash> target and depends on
        # the custom command that generates the merged hexfile.
        add_custom_target(merge_${target}
            DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${target}_merged.hex)

        add_dependencies(merge merge_${target})

        add_custom_target(app_flash_${target}
            COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CONFIG_DIR}/nrfjprog.py ${CMAKE_CURRENT_BINARY_DIR}/${target}.hex --sectorerase
            USES_TERMINAL
            DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${target}.hex)

        add_custom_target(flash_${target}
            COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CONFIG_DIR}/nrfjprog.py ${CMAKE_CURRENT_BINARY_DIR}/${target}_merged.hex --chiperase
            USES_TERMINAL
            DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${target}_merged.hex)

        add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${target}_merged.hex
            COMMAND ${MERGEHEX} -m ${${SOFTDEVICE}_HEX_FILE} ${CMAKE_CURRENT_BINARY_DIR}/${target}.hex -o ${CMAKE_CURRENT_BINARY_DIR}/${target}_merged.hex
            DEPENDS ${target}
            VERBATIM)
    endfunction(add_flash_target)
else ()
    message(STATUS "Could not find nRFx command line tools (`nrfjprog` and `mergehex`).
   See https://infocenter.nordicsemi.com/topic/ug_nrf5x_cltools/UG/cltools/nrf5x_installation.html.
   Flash target will not be supported.")
    function(add_flash_target target)
        # Not supported
    endfunction(add_flash_target)
endif (NRFJPROG AND MERGEHEX AND PYTHON_EXECUTABLE)
