set(WEAK_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/common/src/nrf_mesh_weak.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/src/app_error_weak.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/src/assertion_handler_weak.c"
    CACHE INTERNAL "")
add_subdirectory("common")
add_subdirectory("beaconing")
add_subdirectory("pb_remote")
add_subdirectory("provisioner")

if ((NOT PLATFORM MATCHES "nrf52810") AND (NOT PLATFORM MATCHES "nrf52820"))
    add_subdirectory("sensor")
    add_subdirectory("scene")
    add_subdirectory("light_lightness")
    add_subdirectory("light_lc")
    add_subdirectory("light_ctl")
    add_subdirectory("enocean_switch")
    add_subdirectory("dimming")
    add_subdirectory("lpn")
    add_subdirectory("dfu")
    add_subdirectory("serial")
    add_subdirectory("light_switch")
endif()
