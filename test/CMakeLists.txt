include_directories($<TARGET_PROPERTY:EDyn,INCLUDE_DIRECTORIES>)
add_compile_options($<TARGET_PROPERTY:EDyn,COMPILE_OPTIONS>)

add_executable(vector3 edyn/math/vector3.cpp)

target_link_libraries(vector3 EDyn)

add_test(NAME vector3 COMMAND vector3)