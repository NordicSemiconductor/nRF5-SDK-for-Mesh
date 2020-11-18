set(MESH_STACK_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/mesh_stack.c" CACHE INTERNAL "")

set(MESH_STACK_INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/api" CACHE INTERNAL "")

set(lint_include_dirs
    ${MESH_STACK_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/api
    ${CMAKE_CURRENT_SOURCE_DIR}/../access/api
    ${CMAKE_CURRENT_SOURCE_DIR}/../access/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../bearer/api
    ${CMAKE_CURRENT_SOURCE_DIR}/../bearer/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../prov/api
    ${CMAKE_CURRENT_SOURCE_DIR}/../prov/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../core/api
    ${CMAKE_CURRENT_SOURCE_DIR}/../core/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../dfu/api
    ${CMAKE_CURRENT_SOURCE_DIR}/../dfu/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../examples/templates
    ${CMAKE_CURRENT_SOURCE_DIR}/../../models/foundation/config/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../models/foundation/health/include
    ${${SOFTDEVICE}_INCLUDE_DIRS}
    ${${PLATFORM}_INCLUDE_DIRS}
    ${${nRF5_SDK_VERSION}_INCLUDE_DIRS})

add_pc_lint(mesh_stack_${PLATFORM}
    "${MESH_STACK_SOURCE_FILES}"
    "${lint_include_dirs}"
    "${${PLATFORM}_DEFINES};${${SOFTDEVICE}_DEFINES}")
