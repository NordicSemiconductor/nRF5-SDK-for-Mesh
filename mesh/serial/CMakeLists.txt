set(SERIAL_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/serial_uart.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/serial_bearer.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/serial_handler_common.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/serial_handler_access.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/serial_handler_config.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/serial_handler_openmesh.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/serial.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/serial_handler_prov.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/serial_handler_dfu.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/serial_handler_mesh.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/serial_handler_models.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/serial_handler_device.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/nrf_mesh_serial.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/serial_handler_app.c" CACHE INTERNAL "")

set(SERIAL_INCLUDE_DIRS
    "${SDK_ROOT}/modules/nrfx"
    "${SDK_ROOT}/modules/nrfx/hal"
    "${CMAKE_CURRENT_SOURCE_DIR}/api"
    "${CMAKE_CURRENT_SOURCE_DIR}/include" CACHE INTERNAL "")

set(lint_include_dirs
    ${SERIAL_INCLUDE_DIRS}
    ${${BOARD}_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../examples/templates
    ${CMAKE_CURRENT_SOURCE_DIR}/api
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../prov/api
    ${CMAKE_CURRENT_SOURCE_DIR}/../prov/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../access/api
    ${CMAKE_CURRENT_SOURCE_DIR}/../access/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../core/api
    ${CMAKE_CURRENT_SOURCE_DIR}/../core/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../bearer/api
    ${CMAKE_CURRENT_SOURCE_DIR}/../bearer/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../dfu/api
    ${CMAKE_CURRENT_SOURCE_DIR}/../dfu/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../stack/api
    ${CMAKE_CURRENT_SOURCE_DIR}/../test/include # sdk_config.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../../models/foundation/config/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../models/foundation/health/include
    ${${SOFTDEVICE}_INCLUDE_DIRS}
    ${${PLATFORM}_INCLUDE_DIRS}
    ${${nRF5_SDK_VERSION}_INCLUDE_DIRS})

add_pc_lint(serial_${PLATFORM}
    "${SERIAL_SOURCE_FILES}"
    "${lint_include_dirs}"
    "${${PLATFORM}_DEFINES};${${SOFTDEVICE}_DEFINES};${${BOARD}_DEFINES}")


# TODO: Because of header dependencies (DSM, Access, ..) we cannot compile
# the serial sources as an object "library".
#
# add_library(serial_${PLATFORM} OBJECT ${SERIAL_SRCS})
# target_include_directories(serial_${PLATFORM}
#     PUBLIC
#     ${CMAKE_CURRENT_SOURCE_DIR}/api
#     ${${BOARD}_INCLUDE_DIRS}
#     ${${SOFTDEVICE}_INCLUDE_DIRS}
#     ${${PLATFORM}_INCLUDE_DIRS}
#     PRIVATE
#     ${CMAKE_CURRENT_SOURCE_DIR}/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/../app
#     ${CMAKE_CURRENT_SOURCE_DIR}/../prov/api
#     ${CMAKE_CURRENT_SOURCE_DIR}/../core/api
#     ${CMAKE_CURRENT_SOURCE_DIR}/../core/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/../dfu/api
#     ${CMAKE_CURRENT_SOURCE_DIR}/../dfu/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/../access/api
#     ${CMAKE_CURRENT_SOURCE_DIR}/../access/include)
#
# target_compile_options(serial_${PLATFORM}
#     PRIVATE
#     ${${SOFTDEVICE}_DEFINES}
#     ${${PLATFORM}_DEFINES}
#     ${${ARCH}_DEFINES}
#     ${${BOARD}_DEFINES})
#
# add_dependencies(serial_${PLATFORM}
#     mesh_core_${PLATFORM}
#     prov_provisioner_${PLATFORM}
#     prov_provisioner_${PLATFORM})
