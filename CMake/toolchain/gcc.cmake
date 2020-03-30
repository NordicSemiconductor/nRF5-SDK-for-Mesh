if (CMAKE_HOST_WIN32)
    message(STATUS "Using GCC for Windows (MinGW)")
    set(GCC_PATH "C:/MinGW/bin/gcc.exe" CACHE FILEPATH "MinGW GCC compiler path")
    set(CMAKE_C_COMPILER "${GCC_PATH}")
    set(CMAKE_C_FLAGS_INIT "-mno-ms-bitfields")
else ()
    message(STATUS "Using GCC")
    set(CMAKE_C_COMPILER "gcc")
endif ()

set(data_flags "-ffunction-sections -fdata-sections -fno-strict-aliasing -fno-builtin --short-enums -m32")
set(warning_flags "-Wall -Wextra -Werror=implicit-function-declaration -Wno-format -Wno-unused-parameter -Wno-missing-field-initializers -Wno-expansion-to-defined")

set(CMAKE_C_FLAGS_INIT "--std=gnu99 ${data_flags} ${warning_flags} ${CMAKE_C_FLAGS_INIT}")
set(CMAKE_C_FLAGS_DEBUG_INIT "-O0 -g")
set(CMAKE_EXE_LINKER_FLAGS_INIT "")

set(MATH_LIB m)
