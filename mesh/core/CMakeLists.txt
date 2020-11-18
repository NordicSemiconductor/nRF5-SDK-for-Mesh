set(MESH_CORE_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/internal_event.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/nrf_mesh_configure.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/aes.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/msg_cache.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/transport.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/event.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/packet_buffer.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/flash_manager_defrag.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/fifo.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/nrf_flash.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/net_state.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/mesh_flash.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/nrf_mesh_utils.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/nrf_mesh.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/queue.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/hal.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/aes_cmac.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/timer_scheduler.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/timer.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/long_timer.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/rand.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/nrf_mesh_opt.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/mesh_opt.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/timeslot.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/timeslot_timer.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/bearer_event.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/enc.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/network.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/net_packet.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/msqueue.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/nrf_mesh_keygen.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/cache.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/list.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/log.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/flash_manager.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ccm_soft.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/toolchain.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/replay_cache.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/beacon.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/flash_manager_internal.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/core_tx.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/heartbeat.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/net_beacon.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/mesh_config.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/fsm.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/mesh_config_backend.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/mesh_config_flashman_glue.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/lpn.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/core_tx_lpn.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/mesh_lpn_subman.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/core_tx_local.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/emergency_cache.c"

    # uri.c is not in use.
    # It is an optional feature to include
    # in the build if you want to support generating and parsing of URIs
    # as in Bluetooth CSSv6 Ch. 1.18.
    # "${CMAKE_CURRENT_SOURCE_DIR}/src/uri.c"
    )

set(MESH_CORE_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/api
    ${CMAKE_CURRENT_SOURCE_DIR}/include)

if (EXPERIMENTAL_INSTABURST_ENABLED)
    set(MESH_CORE_SOURCE_FILES ${MESH_CORE_SOURCE_FILES}
        "${CMAKE_CURRENT_SOURCE_DIR}/src/core_tx_instaburst.c")
else()
    set(MESH_CORE_SOURCE_FILES ${MESH_CORE_SOURCE_FILES}
        "${CMAKE_CURRENT_SOURCE_DIR}/src/core_tx_adv.c")
endif()

if (MESH_FEATURE_FRIEND_ENABLED)
    set(MESH_CORE_SOURCE_FILES ${MESH_CORE_SOURCE_FILES}
    "${CMAKE_CURRENT_SOURCE_DIR}/../friend/src/friend.c")
    set(MESH_CORE_INCLUDE_DIRS ${MESH_CORE_INCLUDE_DIRS}
    "${CMAKE_CURRENT_SOURCE_DIR}/../friend/include")
endif()

if (MESH_MEM_BACKEND STREQUAL "stdlib")
    set(MESH_CORE_SOURCE_FILES ${MESH_CORE_SOURCE_FILES}
        "${CMAKE_CURRENT_SOURCE_DIR}/src/mesh_mem_stdlib.c")
elseif (MESH_MEM_BACKEND STREQUAL "packet_mgr")
    set(MESH_CORE_SOURCE_FILES ${MESH_CORE_SOURCE_FILES}
        "${CMAKE_CURRENT_SOURCE_DIR}/src/mesh_mem_packet_mgr.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/packet_mgr.c")

elseif (MESH_MEM_BACKEND STREQUAL "mem_manager")
    set(MESH_CORE_SOURCE_FILES ${MESH_CORE_SOURCE_FILES}
        "${CMAKE_CURRENT_SOURCE_DIR}/src/mesh_mem_mem_manager.c"
        "${SDK_ROOT}/components/libraries/mem_manager/mem_manager.c")
    set(MESH_CORE_INCLUDE_DIRS
        ${MESH_CORE_INCLUDE_DIRS}
        "${SDK_ROOT}/components/libraries/experimental_log"
        "${SDK_ROOT}/components/libraries/experimental_log/src"
        "${SDK_ROOT}/components/libraries/mem_manager")

else ()
    message(FATAL_ERROR "Unknown mesh_mem backend \"${MESH_MEM_BACKEND}\"")
endif ()

# Can only save to cache once without using 'FORCE' modifier
set(MESH_CORE_SOURCE_FILES ${MESH_CORE_SOURCE_FILES} CACHE INTERNAL "")

if (MESH_CORE_LIB_BUILD_ENABLED)
    add_library(mesh_core_${PLATFORM} STATIC ${MESH_CORE_SOURCE_FILES} ${MESH_BEARER_SOURCE_FILES})

    target_include_directories(mesh_core_${PLATFORM} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/api
        ${${SOFTDEVICE}_INCLUDE_DIRS}
        ${${PLATFORM}_INCLUDE_DIRS}
        ${${nRF5_SDK_VERSION}_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/external/rtt/include

        ${SDK_ROOT}/components/libraries/util
        ${SDK_ROOT}/components/libraries/experimental_section_vars

        # TODO: Better encapsulation/separation should keep headers under
        # include private.
        ${CMAKE_CURRENT_SOURCE_DIR}/include

        # TODO: Should not need to include DFU headers in CORE
        ${CMAKE_CURRENT_SOURCE_DIR}/../dfu/api
        ${CMAKE_CURRENT_SOURCE_DIR}/../dfu/include

        # TODO: Should not need to include PROV headers in CORE
        ${CMAKE_CURRENT_SOURCE_DIR}/../prov/api
        ${CMAKE_CURRENT_SOURCE_DIR}/../prov/include

        # TODO: Should not need to include BEARER headers in CORE
        ${CMAKE_CURRENT_SOURCE_DIR}/../bearer/api
        ${CMAKE_CURRENT_SOURCE_DIR}/../bearer/include
        )

    target_compile_options(mesh_core_${PLATFORM} PUBLIC
        ${${ARCH}_DEFINES})

    target_compile_definitions(mesh_core_${PLATFORM} PUBLIC
        ${USER_DEFINITIONS}
        ${${PLATFORM}_DEFINES}
        ${${SOFTDEVICE}_DEFINES})
endif()

if (NRF_MESH_TEST_BUILD)
    set(MESH_CORE_INCLUDE_DIRS ${MESH_CORE_INCLUDE_DIRS} "${CMAKE_SOURCE_DIR}/mesh/test/include" )
endif()

set(MESH_CORE_INCLUDE_DIRS ${MESH_CORE_INCLUDE_DIRS} CACHE INTERNAL "")

set(lint_include_dirs
    ${MESH_CORE_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/../prov/api
    ${CMAKE_CURRENT_SOURCE_DIR}/../prov/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../bearer/api
    ${CMAKE_CURRENT_SOURCE_DIR}/../bearer/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../dfu/api
    ${CMAKE_CURRENT_SOURCE_DIR}/../dfu/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../access/api
    ${CMAKE_CURRENT_SOURCE_DIR}/../test/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../friend/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../gatt/api
    ${CMAKE_CURRENT_SOURCE_DIR}/../gatt/include
    ${CMAKE_SOURCE_DIR}/examples/templates
    ${CMAKE_SOURCE_DIR}/external/rtt/include
    ${${SOFTDEVICE}_INCLUDE_DIRS}
    ${${PLATFORM}_INCLUDE_DIRS}
    ${${nRF5_SDK_VERSION}_INCLUDE_DIRS})

set(LINT_FEATURE_INCLUSION_DEFINES
    "-DMESH_FRIEND_FEATURE_ENABLED=1"
    "-DMESH_FEATURE_LPN_ENABLED=1")

set(lint_optional_feature_include_dirs
    "${CMAKE_CURRENT_SOURCE_DIR}/../friend/include")

add_pc_lint(mesh_core_${PLATFORM}
    "${MESH_CORE_SOURCE_FILES};"
    "${lint_include_dirs};${lint_optional_feature_include_dirs}"
    "${${PLATFORM}_DEFINES};${${SOFTDEVICE}_DEFINES};${LINT_FEATURE_INCLUSION_DEFINES}")
