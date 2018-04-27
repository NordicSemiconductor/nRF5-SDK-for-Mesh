# When compiled in, the examples should "paint" the stack and this can be
# analyze with the stack depth tool.
if (NRF_MESH_STACK_DEPTH)
    add_definitions(-DNRF_MESH_STACK_DEPTH)
endif()
