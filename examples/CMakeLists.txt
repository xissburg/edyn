include_directories($<TARGET_PROPERTY:EDyn,INCLUDE_DIRECTORIES>)
add_compile_options($<TARGET_PROPERTY:EDyn,COMPILE_OPTIONS>)

macro(SETUP_AND_ADD_EXAMPLE EXAMPLE_NAME EXAMPLE_SOURCES)
    add_executable(${EXAMPLE_NAME} ${EXAMPLE_SOURCES})
    target_link_libraries(${EXAMPLE_NAME} PRIVATE EDyn)
    set_property(TARGET ${EXAMPLE_NAME} PROPERTY RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin/examples)
endmacro()

SETUP_AND_ADD_EXAMPLE(hello_world hello_world/hello_world.cpp)
#SETUP_AND_ADD_EXAMPLE(hello_world_mt hello_world_mt/hello_world_mt.cpp)
SETUP_AND_ADD_EXAMPLE(current_pos current_pos/current_pos.cpp)