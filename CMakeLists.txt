cmake_minimum_required(VERSION 3.7.2)

project(EDyn VERSION 0.0.0 LANGUAGES CXX)

#
# Explicit debug build by default
#

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

if(NOT CMAKE_DEBUG_POSTFIX)
  set(CMAKE_DEBUG_POSTFIX "_d")
endif()

#
# Warnings
#

if(MSVC)
  add_compile_options(/W4 /WX)
else()
  add_compile_options(-Wall -Wno-reorder -Wno-long-long -Wimplicit-fallthrough)
endif()

#
# Options
#

option(EDYN_CONFIG_DOUBLE "Use doubles instead of floats." OFF)

set(EDYN_SET_DOUBLE 0)
if(EDYN_CONFIG_DOUBLE)
    set(EDYN_SET_DOUBLE 1)
endif()

option(EDYN_BUILD_EXAMPLES "Build examples." OFF)

option(EDYN_BUILD_TESTS "Build tests with gtest" OFF)

option(ENTT_DISABLE_ASSERT "Disable EnTT assertions for better performance." ON)

if(ENTT_DISABLE_ASSERT)
    add_compile_definitions(ENTT_DISABLE_ASSERT)
endif()

#
# Conan dependencies
#

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

#
# Library
#

set(EDyn_SOURCES
    src/edyn/collision/broadphase.cpp
    src/edyn/collision/sphere_plane_col_alg.cpp
    src/edyn/collision/sphere_sphere_col_alg.cpp
    src/edyn/constraints/contact_constraint.cpp
    src/edyn/constraints/distance_constraint.cpp
    src/edyn/constraints/point_constraint.cpp
    src/edyn/dynamics/island_util.cpp
    src/edyn/dynamics/solver.cpp
    src/edyn/dynamics/world.cpp
    src/edyn/sys/apply_gravity.cpp
    src/edyn/util/rigidbody.cpp
    src/edyn/edyn.cpp
)

if(UNIX)
    set(EDyn_SOURCES
        ${EDyn_SOURCES}
        src/edyn/time/unix/time.cpp
    )
endif()

add_library(EDyn STATIC ${EDyn_SOURCES})

configure_file(${EDyn_SOURCE_DIR}/cmake/in/build_settings.h.in ${EDyn_BINARY_DIR}/include/edyn/build_settings.h @ONLY)
install(FILES ${EDyn_BINARY_DIR}/include/build_settings.h DESTINATION include/edyn)

target_include_directories(EDyn 
    PUBLIC ${EDyn_SOURCE_DIR}/include
    PUBLIC ${EDyn_BINARY_DIR}/include
)

target_include_directories(EDyn SYSTEM 
    PUBLIC ${CONAN_INCLUDE_DIRS_ENTT}
)

target_compile_features(EDyn PUBLIC cxx_std_17)

#
# Examples
#

if(EDYN_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

#
# Tests
#

if(EDYN_BUILD_TESTS)
    find_package(GTest REQUIRED)
    find_library(GMock_LIBRARY gmock)
    find_library(GMockMain_LIBRARY gmock_main)

    enable_testing()
    add_subdirectory(test)
endif()