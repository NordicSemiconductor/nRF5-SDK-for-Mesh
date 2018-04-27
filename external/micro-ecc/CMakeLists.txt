set(uECC_DEFINES
    -DuECC_OPTIMIZATION_LEVEL=2
    -DuECC_SUPPORTS_secp160r1=0
    -DuECC_SUPPORTS_secp192r1=0
    -DuECC_SUPPORTS_secp224r1=0
    -DuECC_SUPPORTS_secp256r1=1
    -DuECC_SUPPORTS_secp256k1=0
    -DuECC_SUPPORT_COMPRESSED_POINT=0)
add_library(uECC_${PLATFORM} STATIC
    "${CMAKE_CURRENT_SOURCE_DIR}/uECC.c")
target_include_directories(uECC_${PLATFORM} PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../mesh/core/include")
target_compile_options(uECC_${PLATFORM} PRIVATE
    ${uECC_DEFINES}
    ${${PLATFORM}_DEFINES}
    ${${ARCH}_DEFINES})

if (TOOLCHAIN STREQUAL "gccarmemb")
    target_compile_options(uECC_${PLATFORM} PRIVATE "-fomit-frame-pointer")
endif()

