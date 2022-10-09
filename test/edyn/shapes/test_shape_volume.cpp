#include "../common/common.hpp"
#include "edyn/util/shape_util.hpp"
#include "edyn/math/shape_volume.hpp"

TEST(test_shape_volume, polyhedron_volume) {
    auto mesh = std::make_shared<edyn::convex_mesh>();
    edyn::make_box_mesh({0.5, 0.5, 0.5}, mesh->vertices,
                        mesh->indices, mesh->faces);
    mesh->initialize();
    auto polyhedron = edyn::polyhedron_shape(mesh);

    ASSERT_SCALAR_EQ(edyn::shape_volume(polyhedron), edyn::scalar(1));

    // Rotate mesh arbitrarily.
    auto q = edyn::quaternion_axis_angle(edyn::normalize(edyn::vector3{3, 8, -1}), edyn::pi * 1.34);
    for (auto &v : mesh->vertices) {
        v = edyn::rotate(q, v);
    }
    for (auto &n : mesh->normals) {
        n = edyn::rotate(q, n);
    }

    // Precision suffers after rotation.
    ASSERT_LE(std::abs(edyn::shape_volume(polyhedron) - edyn::scalar(1)), 0.00001);

    // Move mesh away from the origin.
    for (auto &v : mesh->vertices) {
        v += edyn::vector3{10, -12, 20.889};
    }

    ASSERT_LE(std::abs(edyn::shape_volume(polyhedron) - edyn::scalar(1)), 0.00001);

    // Scale vertices.
    for (auto &v : mesh->vertices) {
        v *= 2;
    }

    ASSERT_LE(std::abs(edyn::shape_volume(polyhedron) - edyn::scalar(8)), 0.0001);
}
