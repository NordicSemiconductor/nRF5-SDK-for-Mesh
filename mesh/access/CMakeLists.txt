# TODO: Because of the weak linkage, compiling ACCESS as a library isn't working.

set(ACCESS_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/access_publish.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/access_publish_retransmission.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/access.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/access_reliable.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/device_state_manager.c" CACHE INTERNAL "")

set(ACCESS_INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/api"
    "${CMAKE_CURRENT_SOURCE_DIR}/include" CACHE INTERNAL "")

set(lint_include_dirs
    ${ACCESS_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/api
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../prov/api
    ${CMAKE_CURRENT_SOURCE_DIR}/../prov/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../bearer/api
    ${CMAKE_CURRENT_SOURCE_DIR}/../bearer/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../core/api
    ${CMAKE_CURRENT_SOURCE_DIR}/../core/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../dfu/api
    ${CMAKE_CURRENT_SOURCE_DIR}/../dfu/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../gatt/api
    ${CMAKE_CURRENT_SOURCE_DIR}/../gatt/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../examples/templates
    ${CMAKE_CURRENT_SOURCE_DIR}/../../models/foundation/config/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../models/foundation/health/include
    ${${SOFTDEVICE}_INCLUDE_DIRS}
    ${${PLATFORM}_INCLUDE_DIRS}
    ${${nRF5_SDK_VERSION}_INCLUDE_DIRS})

add_pc_lint(access_${PLATFORM}
    "${ACCESS_SOURCE_FILES}"
    "${lint_include_dirs}"
    "${${PLATFORM}_DEFINES};${${SOFTDEVICE}_DEFINES}")
