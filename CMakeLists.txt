cmake_minimum_required(VERSION 3.12.4)

set(Edyn_MAIN_PROJECT ON)
if(DEFINED PROJECT_SOURCE_DIR)
    set(Edyn_MAIN_PROJECT OFF)
endif()

project(Edyn VERSION 0.2.0 LANGUAGES CXX)

include(CMakeDependentOption)

# Options
option(EDYN_CONFIG_DOUBLE "Use doubles instead of floats" OFF)
option(EDYN_INSTALL "Enable installation of Edyn" ${Edyn_MAIN_PROJECT})
option(EDYN_BUILD_EXAMPLES "Build examples" ${Edyn_MAIN_PROJECT})
option(EDYN_BUILD_TESTS "Build tests with gtest" OFF)
option(EDYN_DISABLE_ASSERT "Disable assertions in Edyn for better performance." OFF)
cmake_dependent_option(EDYN_ENABLE_SANITIZER "Enable address sanitizer." OFF "NOT MSVC" OFF)

if(NOT CMAKE_DEBUG_POSTFIX)
  set(CMAKE_DEBUG_POSTFIX "_d")
endif()

find_package(EnTT REQUIRED)

configure_file(cmake/in/build_settings.h.in include/edyn/build_settings.h @ONLY)

add_library(Edyn
    cmake/in/build_settings.h.in
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
    src/edyn/collision/collide/collide_capsule_mesh.cpp
    src/edyn/collision/collide/collide_box_box.cpp
    src/edyn/collision/collide/collide_box_plane.cpp
    src/edyn/collision/collide/collide_capsule_box.cpp
    src/edyn/collision/collide/collide_cylinder_box.cpp
    src/edyn/collision/collide/collide_sphere_box.cpp
    src/edyn/collision/collide/collide_cylinder_mesh.cpp
    src/edyn/collision/collide/collide_sphere_mesh.cpp
    src/edyn/collision/collide/collide_box_mesh.cpp
    src/edyn/collision/collide/collide_polyhedron_capsule.cpp
    src/edyn/collision/collide/collide_polyhedron_cylinder.cpp
    src/edyn/collision/collide/collide_polyhedron_box.cpp
    src/edyn/collision/collide/collide_polyhedron_plane.cpp
    src/edyn/collision/collide/collide_polyhedron_polyhedron.cpp
    src/edyn/collision/collide/collide_polyhedron_sphere.cpp
    src/edyn/collision/collide/collide_polyhedron_mesh.cpp
    src/edyn/collision/collide/collide_compound_compound.cpp
    src/edyn/collision/collide/collide_compound_plane.cpp
    src/edyn/collision/collide/collide_compound_mesh.cpp
    src/edyn/collision/should_collide.cpp
    src/edyn/collision/collision_result.cpp
    src/edyn/collision/raycast.cpp
    src/edyn/constraints/contact_constraint.cpp
    src/edyn/constraints/distance_constraint.cpp
    src/edyn/constraints/soft_distance_constraint.cpp
    src/edyn/constraints/point_constraint.cpp
    src/edyn/constraints/hinge_constraint.cpp
    src/edyn/constraints/generic_constraint.cpp
    src/edyn/constraints/cvjoint_constraint.cpp
    src/edyn/constraints/cone_constraint.cpp
    src/edyn/constraints/null_constraint.cpp
    src/edyn/constraints/gravity_constraint.cpp
    src/edyn/dynamics/solver.cpp
    src/edyn/dynamics/restitution_solver.cpp
    src/edyn/sys/update_aabbs.cpp
    src/edyn/sys/update_rotated_meshes.cpp
    src/edyn/sys/update_inertias.cpp
    src/edyn/sys/update_presentation.cpp
    src/edyn/sys/update_origins.cpp
    src/edyn/util/rigidbody.cpp
    src/edyn/util/constraint_util.cpp
    src/edyn/util/shape_util.cpp
    src/edyn/util/aabb_util.cpp
    src/edyn/util/moment_of_inertia.cpp
    src/edyn/util/shape_volume.cpp
    src/edyn/util/collision_util.cpp
    src/edyn/shapes/triangle_mesh.cpp
    src/edyn/shapes/paged_triangle_mesh.cpp
    src/edyn/util/triangle_util.cpp
    src/edyn/util/ragdoll.cpp
    src/edyn/util/exclude_collision.cpp
    src/edyn/util/make_reg_op_builder.cpp
    src/edyn/shapes/box_shape.cpp
    src/edyn/shapes/cylinder_shape.cpp
    src/edyn/shapes/polyhedron_shape.cpp
    src/edyn/shapes/convex_mesh.cpp
    src/edyn/shapes/compound_shape.cpp
    src/edyn/parallel/entity_graph.cpp
    src/edyn/parallel/job_queue.cpp
    src/edyn/parallel/job_dispatcher.cpp
    src/edyn/parallel/job_scheduler.cpp
    src/edyn/parallel/job_queue_scheduler.cpp
    src/edyn/parallel/island_worker.cpp
    src/edyn/parallel/island_coordinator.cpp
    src/edyn/parallel/island_worker_context.cpp
    src/edyn/parallel/map_child_entity.cpp
    src/edyn/serialization/paged_triangle_mesh_s11n.cpp
    src/edyn/networking/context/client_network_context.cpp
    src/edyn/networking/context/server_network_context.cpp
    src/edyn/networking/sys/server_side.cpp
    src/edyn/networking/sys/client_side.cpp
    src/edyn/networking/networking.cpp
    src/edyn/networking/sys/update_aabbs_of_interest.cpp
    src/edyn/networking/extrapolation_job.cpp
    src/edyn/networking/util/pool_snapshot.cpp
    src/edyn/networking/util/clock_sync.cpp
    src/edyn/networking/util/process_update_entity_map_packet.cpp
    src/edyn/context/settings.cpp
    src/edyn/edyn.cpp
    src/edyn/time/common/time.cpp
)
add_library(Edyn::Edyn ALIAS Edyn)

if(UNIX)
    target_sources(Edyn PRIVATE
        src/edyn/time/unix/time.cpp
    )
endif()

if(UNIX AND NOT APPLE)
    target_link_libraries(Edyn
        PUBLIC pthread
    )
endif()

if(WIN32)
    target_sources(Edyn PRIVATE
        src/edyn/time/windows/time.cpp
    )
    target_link_libraries(Edyn
        PUBLIC winmm
    )
endif()

target_include_directories(Edyn
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

set_target_properties(Edyn PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

target_compile_features(Edyn PUBLIC cxx_std_17)
target_compile_definitions(Edyn
    PUBLIC
        $<$<CONFIG:Debug>:EDYN_DEBUG>
        $<$<BOOL:${EDYN_DISABLE_ASSERT}>:EDYN_DISABLE_ASSERT>
    PRIVATE
        $<$<BOOL:${EDYN_DISABLE_ASSERT}>:ENTT_DISABLE_ASSERT>
)

target_compile_options(Edyn PUBLIC
    $<$<BOOL:${EDYN_ENABLE_SANITIZER}>:-fsanitize=address -fsanitize=undefined -fno-omit-frame-pointer>
)

target_link_libraries(Edyn
    PUBLIC
        EnTT::EnTT
)

target_link_options(Edyn
    PUBLIC
        $<$<BOOL:${EDYN_ENABLE_SANITIZER}>:-fsanitize=address -fsanitize=undefined>
)

if(MSVC)
    target_compile_options(Edyn PRIVATE /W4)
else()
    target_compile_options(Edyn PRIVATE -Wall -Wno-reorder -Wno-long-long -Wimplicit-fallthrough)
endif()

if(EDYN_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

if(EDYN_BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()

if(EDYN_INSTALL)
    include(GNUInstallDirs)
    install(
        TARGETS Edyn
        EXPORT EdynTargets
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
    )
    install(
        DIRECTORY include/edyn/ src/edyn/ ${PROJECT_BINARY_DIR}/include/edyn/
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/edyn"
        FILES_MATCHING
            PATTERN "*.h"
            PATTERN "*.hpp"
    )
    install(
        EXPORT EdynTargets
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
        NAMESPACE "Edyn::"
        FILE "Edyn-targets.cmake"
    )
    install(
        FILES
            cmake/Edyn-config.cmake
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
    )
    install(
        FILES
            LICENSE
        DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/licenses/${PROJECT_NAME}"
    )
    if(MSVC AND BUILD_SHARED_LIBS)
        install(
            FILES $<TARGET_PDB_FILE:Edyn>
            DESTINATION "${CMAKE_INSTALL_BINDIR}"
            OPTIONAL
        )
    endif()
endif()
