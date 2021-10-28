macro(SETUP_AND_ADD_EXAMPLE EXAMPLE_NAME EXAMPLE_SOURCES)
    add_executable(${EXAMPLE_NAME} ${EXAMPLE_SOURCES})
    target_link_libraries(${EXAMPLE_NAME} PRIVATE Edyn::Edyn)

    set_property(TARGET ${EXAMPLE_NAME} PROPERTY RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin/examples)
endmacro()

SETUP_AND_ADD_EXAMPLE(hello_world hello_world/hello_world.cpp)
SETUP_AND_ADD_EXAMPLE(current_pos current_pos/current_pos.cpp)
