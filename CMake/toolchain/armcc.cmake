set(DEFAULT_KEIL_PATH "C:/Keil_v5/ARM/ARMCC/bin")

find_program(ARMCC_C_COMPILER armcc
    PATHS "${DEFAULT_KEIL_PATH}" ENV PATH NO_DEFAULT_PATH)
find_program(ARMCC_ASM_COMPILER armasm
    PATHS "${DEFAULT_KEIL_PATH}" ENV PATH NO_DEFAULT_PATH)
find_program(ARMCC_LINKER armlink
    PATHS "${DEFAULT_KEIL_PATH}" ENV PATH NO_DEFAULT_PATH)
find_program(ARMCC_FROMELF fromelf
    PATHS "${DEFAULT_KEIL_PATH}" ENV PATH NO_DEFAULT_PATH)

if (NOT (ARMCC_C_COMPILER AND ARMCC_ASM_COMPILER AND ARMCC_LINKER))
    message(FATAL_ERROR "Could not find ARMCC toolchain.")
endif ()

option(ARMCC_ENABLE_MICROLIB ON "Enable the ARM microlib?")

set(CMAKE_C_COMPILER ${ARMCC_C_COMPILER})
set(CMAKE_ASM_COMPILER ${ARMCC_ASM_COMPILER})
set(CMAKE_LINKER ${ARMCC_LINKER})
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_C_FLAGS_INIT "--c99")
set(CMAKE_C_FLAGS_RELEASE "-O3 --no_debug" CACHE STRING "")
set(CMAKE_C_FLAGS_DEBUG "-O1 --debug" CACHE STRING "")
set(CMAKE_C_FLAGS_MINSIZEREL "-Ospace --debug" CACHE STRING "")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-O3 --debug" CACHE STRING "")

set(CMAKE_EXE_LINKER_FLAGS
    "--map --xref --summary_stderr --info summarysizes --info stack --callgraph --symbols --info sizes --info totals --info unused --info veneers")
# 6304: Suppress the "duplicate input" warning caused by circular linking by CMake
# 6330: Supress undefined symbol warning
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --diag_suppress 6304,6330")
set(CMAKE_ASM_FLAGS "--cpreproc --apcs=interwork  --predefine \"__HEAP_SIZE SETA 1024\"")

set(MATH_LIB)

set(cortex-m0_DEFINES
    --cpu=Cortex-M0)

set(cortex-m4_DEFINES
    --cpu=Cortex-M4)

set(cortex-m4f_DEFINES
    --cpu=Cortex-M4.fp)

if (ARMCC_ENABLE_MICROLIB)
    set(CMAKE_EXE_LINKER_FLAGS "--library_type=microlib ${CMAKE_EXE_LINKER_FLAGS}")
    set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} --predefine \"__MICROLIB SETL {TRUE}\"")
endif()

function (set_target_link_options target_name linker_file)
    set_target_properties(${target_name} PROPERTIES LINK_FLAGS
        "${${ARCH}_DEFINES} --scatter \"${linker_file}.sct\"")
endfunction (set_target_link_options)

function (create_hex executable)
    add_custom_command(
        TARGET ${executable}
        POST_BUILD
        COMMAND ${ARMCC_FROMELF} --i32combined -o ${CMAKE_CURRENT_BINARY_DIR}/${executable}.hex ${CMAKE_CURRENT_BINARY_DIR}/${executable}.elf
        BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/${executable}.hex)
endfunction(create_hex)

# We have to cache the variable before the enable_language() call.
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS}" CACHE STRING "")
