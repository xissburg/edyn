cmake_minimum_required(VERSION 3.12.4)

project(Edyn VERSION 0.0.0 LANGUAGES CXX)

#
# Options
#
option(EDYN_CONFIG_DOUBLE "Use doubles instead of floats" OFF)

if (${CMAKE_CURRENT_SOURCE_DIR} STREQUAL ${CMAKE_SOURCE_DIR})
    option(EDYN_BUILD_EXAMPLES "Build examples" ON)
    option(EDYN_BUILD_TESTS "Build tests with gtest" ON)
else()
    option(EDYN_BUILD_EXAMPLES "Build examples" OFF)
    option(EDYN_BUILD_TESTS "Build tests with gtest" OFF)
endif()

if(NOT CMAKE_DEBUG_POSTFIX)
  set(CMAKE_DEBUG_POSTFIX "_d")
endif()

if(MSVC)
    add_compile_options(/W4 /WX)
else()
    add_compile_options(-Wall -Wno-reorder -Wno-long-long -Wimplicit-fallthrough)
endif()


include(${CMAKE_BINARY_DIR}/conan_paths.cmake OPTIONAL)

find_package(EnTT REQUIRED)

set(Edyn_SOURCES
    src/edyn/math/geom.cpp
    src/edyn/math/quaternion.cpp
    src/edyn/collision/broadphase_main.cpp
    src/edyn/collision/broadphase_worker.cpp
    src/edyn/collision/narrowphase.cpp
    src/edyn/collision/contact_manifold_map.cpp
    src/edyn/collision/dynamic_tree.cpp
    src/edyn/collision/collide/collide_sphere_sphere.cpp
    src/edyn/collision/collide/collide_sphere_plane.cpp
    src/edyn/collision/collide/collide_cylinder_cylinder.cpp
    src/edyn/collision/collide/collide_cylinder_plane.cpp
    src/edyn/collision/collide/collide_cylinder_sphere.cpp
    src/edyn/collision/collide/collide_capsule_capsule.cpp
    src/edyn/collision/collide/collide_capsule_cylinder.cpp
    src/edyn/collision/collide/collide_capsule_plane.cpp
    src/edyn/collision/collide/collide_capsule_sphere.cpp
    src/edyn/collision/collide/collide_sphere_mesh.cpp
    src/edyn/collision/collide/collide_capsule_mesh.cpp
    src/edyn/collision/collide/collide_cylinder_mesh.cpp
    src/edyn/collision/collide/collide_sphere_paged_mesh.cpp
    src/edyn/collision/collide/collide_capsule_paged_mesh.cpp
    src/edyn/collision/collide/collide_cylinder_paged_mesh.cpp
    src/edyn/collision/collide/collide_box_box.cpp
    src/edyn/collision/collide/collide_box_mesh.cpp
    src/edyn/collision/collide/collide_box_paged_mesh.cpp
    src/edyn/collision/collide/collide_box_plane.cpp
    src/edyn/collision/collide/collide_capsule_box.cpp
    src/edyn/collision/collide/collide_cylinder_box.cpp
    src/edyn/collision/collide/collide_sphere_box.cpp
    src/edyn/collision/collide/collide_cylinder_triangle.cpp
    src/edyn/collision/collide/collide_sphere_triangle.cpp
    src/edyn/collision/collide/collide_box_triangle.cpp
    src/edyn/collision/collide/collide_capsule_polyhedron.cpp
    src/edyn/collision/collide/collide_cylinder_polyhedron.cpp
    src/edyn/collision/collide/collide_polyhedron_box.cpp
    src/edyn/collision/collide/collide_polyhedron_mesh.cpp
    src/edyn/collision/collide/collide_polyhedron_paged_mesh.cpp
    src/edyn/collision/collide/collide_polyhedron_plane.cpp
    src/edyn/collision/collide/collide_polyhedron_polyhedron.cpp
    src/edyn/collision/collide/collide_sphere_polyhedron.cpp
    src/edyn/collision/collide/collide_polyhedron_triangle.cpp
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

add_library(Edyn)
target_sources(Edyn PRIVATE ${Edyn_SOURCES})
target_link_libraries(Edyn PUBLIC EnTT::EnTT)

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
target_compile_definitions(Edyn PUBLIC $<$<CONFIG:Debug>:EDYN_DEBUG>)

if(EDYN_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

if(EDYN_BUILD_TESTS)
    find_package(GTest REQUIRED)
    find_library(GMock_LIBRARY gmock)
    find_library(GMockMain_LIBRARY gmock_main)

    enable_testing()
    add_subdirectory(test)
endif()