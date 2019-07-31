if (BUILD_HOST)
    add_subdirectory("test")
else ()
    add_subdirectory("stack")
    add_subdirectory("access")
    add_subdirectory("bootloader")
    add_subdirectory("bearer")
    add_subdirectory("core")
    add_subdirectory("dfu")
    add_subdirectory("prov")
    add_subdirectory("serial")
    add_subdirectory("gatt")
    add_subdirectory("friend")

    set(MESH_INCLUDE_DIRS
        ${MESH_STACK_INCLUDE_DIRS}
        ${MESH_CORE_INCLUDE_DIRS}
        ${ACCESS_INCLUDE_DIRS}
        ${DFU_INCLUDE_DIRS}
        ${PROV_INCLUDE_DIRS}
        ${MESH_BEARER_INCLUDE_DIRS}
        ${MESH_GATT_INCLUDE_DIRS}
        ${MESH_FRIEND_INCLUDE_DIRS}
        CACHE INTERNAL "")
endif (BUILD_HOST)
