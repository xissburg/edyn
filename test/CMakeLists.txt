include(GoogleTest)
find_package(GTest REQUIRED)

macro(setup_and_add_test TEST_NAME TEST_SOURCE)
    add_executable(${TEST_NAME} ${TEST_SOURCE})
    target_link_libraries(${TEST_NAME} PRIVATE Edyn::Edyn GTest::gtest_main)
    set_property(TARGET ${TEST_NAME} PROPERTY RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin/test)
    gtest_discover_tests(${TEST_NAME} PROPERTIES TIMEOUT 5)
endmacro()

setup_and_add_test(vector3 edyn/math/test_vector3.cpp)
setup_and_add_test(matrix3x3 edyn/math/test_matrix3x3.cpp)
setup_and_add_test(triangle_mesh_serialization edyn/serialization/test_triangle_mesh_s11n.cpp)
setup_and_add_test(apply_gravity edyn/sys/test_apply_gravity.cpp)
setup_and_add_test(job_dispatcher edyn/parallel/test_job_dispatcher.cpp)
setup_and_add_test(entity_graph edyn/parallel/test_entity_graph.cpp)
setup_and_add_test(std_serialization edyn/serialization/test_std_s11n.cpp)
setup_and_add_test(geom edyn/math/test_geom.cpp)
setup_and_add_test(math edyn/math/test_math.cpp)
setup_and_add_test(collision edyn/collision/test_collision.cpp)
setup_and_add_test(collision_exclusion edyn/collision/test_exclusion.cpp)
setup_and_add_test(shape_volume edyn/shapes/test_shape_volume.cpp)
setup_and_add_test(centroid edyn/shapes/test_centroid.cpp)
setup_and_add_test(trimesh edyn/shapes/test_trimesh.cpp)
setup_and_add_test(paged_trimesh edyn/shapes/test_paged_trimesh.cpp)
setup_and_add_test(set_shape edyn/shapes/test_set_shape.cpp)
setup_and_add_test(broadphase edyn/collision/test_broadphase.cpp)
setup_and_add_test(raycast edyn/collision/test_raycast.cpp)
setup_and_add_test(tuple_util edyn/util/test_tuple_util.cpp)
setup_and_add_test(registry_operation edyn/util/test_registry_operation.cpp)
setup_and_add_test(issue76 edyn/issues/issue76.cpp)
setup_and_add_test(networking_import_export edyn/networking/test_net_imp_exp.cpp)
setup_and_add_test(input_state_history edyn/networking/test_input_state_history.cpp)
setup_and_add_test(rigidbody_kind edyn/util/test_change_rigidbody_kind.cpp)
setup_and_add_test(clear_rigidbody edyn/util/test_clear_rigidbody.cpp)
setup_and_add_test(issue128 edyn/issues/issue128.cpp)
