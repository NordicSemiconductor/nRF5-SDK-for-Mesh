set(MESH_APP_TIMER_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/app_timer_mesh.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/app_timer_workaround.c" 
    CACHE INTERNAL "")

set(target_include_dirs
    ${${nRF5_SDK_VERSION}_INCLUDE_DIRS})    