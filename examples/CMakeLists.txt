macro(SETUP_AND_ADD_EXAMPLE EXAMPLE_NAME EXAMPLE_SOURCES)
    add_executable(${EXAMPLE_NAME} ${EXAMPLE_SOURCES})
    target_compile_features(${EXAMPLE_NAME} PUBLIC cxx_std_17)

    target_link_libraries(${EXAMPLE_NAME}
        Edyn::Edyn
        EnTT::EnTT
    )

    if (UNIX AND NOT APPLE)
        target_link_libraries(${EXAMPLE_NAME}
            dl
            pthread
        )
    endif ()

    if (WIN32)
        target_link_libraries(${EXAMPLE_NAME} winmm Ws2_32)
    endif ()

    set_property(TARGET ${EXAMPLE_NAME} PROPERTY RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin/examples)
endmacro()

SETUP_AND_ADD_EXAMPLE(hello_world hello_world/hello_world.cpp)
SETUP_AND_ADD_EXAMPLE(current_pos current_pos/current_pos.cpp)
