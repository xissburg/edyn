cmake_minimum_required(VERSION 3.0)
project(test_package CXX)

include("${CMAKE_BINARY_DIR}/conanbuildinfo.cmake")
conan_basic_setup(TARGETS)

find_package(Edyn REQUIRED)

add_compile_options(
    "$<IF:$<CXX_COMPILER_ID:MSVC>,/W4,-Wall;-Wextra>"
)

enable_testing()
add_subdirectory(../examples examples)
add_subdirectory(../test test)
