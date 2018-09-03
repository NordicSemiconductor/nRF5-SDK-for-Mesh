set(WEAK_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/nrf_mesh_weak.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/src/app_error_weak.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/src/assertion_handler_weak.c"
    CACHE INTERNAL "")
add_subdirectory("common")
add_subdirectory("enocean_switch")
add_subdirectory("beaconing")
add_subdirectory("light_switch")
add_subdirectory("pb_remote")
add_subdirectory("experimental_dimming")

if (NOT PLATFORM MATCHES "nrf52810")
    add_subdirectory("dfu")
    add_subdirectory("serial")
endif()
