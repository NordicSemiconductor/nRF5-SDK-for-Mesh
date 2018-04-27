set(srcs
  "${CMAKE_CURRENT_SOURCE_DIR}/src/SEGGER_RTT.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/SEGGER_RTT_printf.c")

set(include_directories
  "${CMAKE_CURRENT_SOURCE_DIR}/include")

add_library(rtt_${PLATFORM} STATIC ${srcs})
target_include_directories(rtt_${PLATFORM} PUBLIC
    ${include_directories})
target_compile_options(rtt_${PLATFORM} PRIVATE
    ${${ARCH}_DEFINES})
