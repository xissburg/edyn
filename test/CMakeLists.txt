include_directories($<TARGET_PROPERTY:EDyn,INCLUDE_DIRECTORIES>)
add_compile_options($<TARGET_PROPERTY:EDyn,COMPILE_OPTIONS>)

macro(SETUP_AND_ADD_TEST TEST_NAME TEST_SOURCE)
    add_executable(${TEST_NAME} ${TEST_SOURCE})
    target_link_libraries(${TEST_NAME} PRIVATE EDyn GTest::Main)
    set_property(TARGET ${TEST_NAME} PROPERTY RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin/test)
    gtest_discover_tests(${TEST_NAME})
endmacro()

SETUP_AND_ADD_TEST(vector3 edyn/math/vector3.cpp)
SETUP_AND_ADD_TEST(integrate_linvel edyn/sys/integrate_linvel.cpp)
SETUP_AND_ADD_TEST(integrate_linacc edyn/sys/integrate_linacc.cpp)