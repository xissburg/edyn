cmake_minimum_required(VERSION 3.12.4)

project(Edyn VERSION 0.0.0 LANGUAGES CXX)

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

set(Edyn_SOURCES
    src/edyn/math/geom.cpp
    src/edyn/math/quaternion.cpp
    src/edyn/collision/broadphase_main.cpp
    src/edyn/collision/broadphase_worker.cpp
    src/edyn/collision/narrowphase.cpp
    src/edyn/collision/contact_manifold_map.cpp
    src/edyn/collision/dynamic_tree.cpp
    src/edyn/collision/collide_sphere_sphere.cpp
    src/edyn/collision/collide_sphere_plane.cpp
    src/edyn/collision/collide_cylinder_cylinder.cpp
    src/edyn/collision/collide_cylinder_plane.cpp
    src/edyn/collision/collide_cylinder_sphere.cpp
    src/edyn/collision/collide_capsule_capsule.cpp
    src/edyn/collision/collide_capsule_cylinder.cpp
    src/edyn/collision/collide_capsule_plane.cpp
    src/edyn/collision/collide_capsule_sphere.cpp
    src/edyn/collision/collide_sphere_mesh.cpp
    src/edyn/collision/collide_capsule_mesh.cpp
    src/edyn/collision/collide_cylinder_mesh.cpp
    src/edyn/collision/collide_sphere_paged_mesh.cpp
    src/edyn/collision/collide_capsule_paged_mesh.cpp
    src/edyn/collision/collide_cylinder_paged_mesh.cpp
    src/edyn/collision/collide_box_box.cpp
    src/edyn/collision/collide_box_mesh.cpp
    src/edyn/collision/collide_box_paged_mesh.cpp
    src/edyn/collision/collide_box_plane.cpp
    src/edyn/collision/collide_capsule_box.cpp
    src/edyn/collision/collide_cylinder_box.cpp
    src/edyn/collision/collide_sphere_box.cpp
    src/edyn/collision/collide_cylinder_triangle.cpp
    src/edyn/collision/collide_sphere_triangle.cpp
    src/edyn/collision/collide_box_triangle.cpp
    src/edyn/collision/collide_box_polyhedron.cpp
    src/edyn/collision/collide_capsule_polyhedron.cpp
    src/edyn/collision/collide_cylinder_polyhedron.cpp
    src/edyn/collision/collide_polyhedron_mesh.cpp
    src/edyn/collision/collide_polyhedron_paged_mesh.cpp
    src/edyn/collision/collide_polyhedron_plane.cpp
    src/edyn/collision/collide_polyhedron_polyhedron.cpp
    src/edyn/collision/collide_sphere_polyhedron.cpp
    src/edyn/collision/collision_result.cpp
    src/edyn/constraints/contact_constraint.cpp
    src/edyn/constraints/distance_constraint.cpp
    src/edyn/constraints/soft_distance_constraint.cpp
    src/edyn/constraints/point_constraint.cpp
    src/edyn/constraints/hinge_constraint.cpp
    src/edyn/constraints/generic_constraint.cpp
    src/edyn/dynamics/solver.cpp
    src/edyn/dynamics/world.cpp
    src/edyn/sys/apply_gravity.cpp
    src/edyn/sys/update_aabbs.cpp
    src/edyn/sys/update_rotated_meshes.cpp
    src/edyn/util/rigidbody.cpp
    src/edyn/util/constraint_util.cpp
    src/edyn/util/shape_util.cpp
    src/edyn/util/moment_of_inertia.cpp
    src/edyn/shapes/triangle_mesh.cpp
    src/edyn/shapes/paged_triangle_mesh.cpp
    src/edyn/shapes/triangle_shape.cpp
    src/edyn/shapes/box_shape.cpp
    src/edyn/shapes/cylinder_shape.cpp
    src/edyn/shapes/polyhedron_shape.cpp
    src/edyn/shapes/convex_mesh.cpp
    src/edyn/parallel/entity_graph.cpp
    src/edyn/parallel/job_queue.cpp
    src/edyn/parallel/job_dispatcher.cpp
    src/edyn/parallel/job_scheduler.cpp
    src/edyn/parallel/job_queue_scheduler.cpp
    src/edyn/parallel/island_worker.cpp
    src/edyn/parallel/island_coordinator.cpp
    src/edyn/parallel/island_delta.cpp
    src/edyn/parallel/island_delta_builder.cpp
    src/edyn/parallel/island_worker_context.cpp
    src/edyn/parallel/external_system.cpp
    src/edyn/serialization/paged_triangle_mesh_s11n.cpp
    src/edyn/edyn.cpp
    src/edyn/init.cpp
)

if(UNIX)
    set(Edyn_SOURCES
        ${Edyn_SOURCES}
        src/edyn/time/unix/time.cpp
    )
endif()

add_library(Edyn STATIC ${Edyn_SOURCES})

configure_file(${Edyn_SOURCE_DIR}/cmake/in/build_settings.h.in ${Edyn_BINARY_DIR}/include/edyn/build_settings.h @ONLY)
install(FILES ${Edyn_BINARY_DIR}/include/build_settings.h DESTINATION include/edyn)

target_include_directories(Edyn 
    PUBLIC ${Edyn_SOURCE_DIR}/include
    PUBLIC ${Edyn_BINARY_DIR}/include
)

target_include_directories(Edyn SYSTEM 
    PUBLIC ${CONAN_INCLUDE_DIRS_ENTT}
)

target_compile_features(Edyn PUBLIC cxx_std_17)

#
# Sanitize
#

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    #target_compile_options(Edyn PUBLIC -D_GLIBCXX_DEBUG -fsanitize=address -fno-omit-frame-pointer -fsanitize=undefined)
    #target_link_libraries(Edyn -fsanitize=address -fsanitize=undefined)
endif()

#
# Examples
#

if(EDYN_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

#
# Command-line tool
#



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