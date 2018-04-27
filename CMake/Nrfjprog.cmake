find_program(NRFJPROG
    nrfjprog)

find_program(MERGEHEX
    mergehex)

if (NRFJPROG AND MERGEHEX)
    function(add_flash_target target)
        add_custom_target(flash_${target}
            # COMMAND ${MERGEHEX} -m ${${SOFTDEVICE}_HEX_FILE} ${target}.hex -o ${target}_${SOFTDEVICE}.hex
            COMMAND ${NRFJPROG} -f ${${PLATFORM}_FAMILY} --eraseall
            COMMAND ${NRFJPROG} -f ${${PLATFORM}_FAMILY} --program ${${SOFTDEVICE}_HEX_FILE}
            COMMAND ${NRFJPROG} -f ${${PLATFORM}_FAMILY} --program ${target}.hex -r
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            DEPENDS ${target})
    endfunction(add_flash_target)
else ()
    message(STATUS "Flash target will not be supported.")
    function(add_flash_target target)
        # Not supported
    endfunction(add_flash_target)
endif (NRFJPROG AND MERGEHEX)
