# TODO: Because of the weak linkage, compiling provisioning as a library isn't working.
set(PROV_COMMON_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/nrf_mesh_prov.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/provisioning.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/prov_beacon.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/prov_utils.c" CACHE INTERNAL "")

set(PROV_PROVISIONEE_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/prov_provisionee.c" CACHE INTERNAL "")

set(PROV_PROVISIONER_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/prov_provisioner.c" CACHE INTERNAL "")

set(PROV_BEARER_ADV_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/prov_bearer_adv.c" CACHE INTERNAL "")

set(PROV_BEARER_GATT_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/nrf_mesh_prov_bearer_gatt.c" CACHE INTERNAL "")

set(PROV_INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/api"
    "${CMAKE_CURRENT_SOURCE_DIR}/include" CACHE INTERNAL "")

set(lint_include_dirs
    ${PROV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/api
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../core/api
    ${${SOFTDEVICE}_INCLUDE_DIRS}
    ${${PLATFORM}_INCLUDE_DIRS}
    ${${nRF5_SDK_VERSION}_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/../core/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../gatt/api
    ${CMAKE_CURRENT_SOURCE_DIR}/../gatt/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../bearer/api
    ${CMAKE_CURRENT_SOURCE_DIR}/../bearer/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../dfu/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../dfu/api
    ${CMAKE_CURRENT_SOURCE_DIR}/../test/include # sdk_config.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../../external/micro-ecc)

add_pc_lint(prov_${PLATFORM}
    "${PROV_PROVISIONER_SOURCE_FILES};${PROV_PROVISIONEE_SOURCE_FILES};${PROV_COMMON_SOURCE_FILES};${PROV_BEARER_ADV_SOURCE_FILES};${PROV_BEARER_GATT_SOURCE_FILES}"
    "${lint_include_dirs}"
    "${${PLATFORM}_DEFINES};${${SOFTDEVICE}_DEFINES}")
