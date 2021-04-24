#include "../common/common.hpp"
#include "edyn/util/shape_util.hpp"

TEST(test_centroid, mesh_centroid) {
    auto mesh = edyn::convex_mesh{};
    edyn::make_box_mesh({0.5, 0.5, 0.5}, mesh.vertices, 
                        mesh.indices, mesh.faces);
    mesh.initialize();

    auto center = edyn::mesh_centroid(mesh.vertices, mesh.indices, mesh.faces);

    ASSERT_SCALAR_EQ(center.x, edyn::scalar(0));
    ASSERT_SCALAR_EQ(center.y, edyn::scalar(0));
    ASSERT_SCALAR_EQ(center.z, edyn::scalar(0));

    // Rotate mesh arbitrarily.
    auto q = edyn::quaternion_axis_angle(edyn::normalize(edyn::vector3{-2, 0.22, 7}), edyn::pi * 2.71);
    for (auto &v : mesh.vertices) {
        v = edyn::rotate(q, v);
    }
    for (auto &n : mesh.normals) {
        n = edyn::rotate(q, n);
    }

    center = edyn::mesh_centroid(mesh.vertices, mesh.indices, mesh.faces);

    ASSERT_LE(std::abs(center.x), edyn::scalar(0.0001));
    ASSERT_LE(std::abs(center.y), edyn::scalar(0.0001));
    ASSERT_LE(std::abs(center.z), edyn::scalar(0.0001));

    // Move mesh away from the origin.
    auto position = edyn::vector3{-9.8, 1.85, 12.13};
    for (auto &v : mesh.vertices) {
        v += position;
    }

    center = edyn::mesh_centroid(mesh.vertices, mesh.indices, mesh.faces);

    ASSERT_LE(std::abs(center.x - position.x), edyn::scalar(0.0001));
    ASSERT_LE(std::abs(center.y - position.y), edyn::scalar(0.0001));
    ASSERT_LE(std::abs(center.z - position.z), edyn::scalar(0.0001));    

    // Scale vertices.
    auto scale = edyn::scalar(2);

    for (auto &v : mesh.vertices) {
        v *= scale;
    }

    center = edyn::mesh_centroid(mesh.vertices, mesh.indices, mesh.faces);

    ASSERT_LE(std::abs(center.x - position.x * scale), edyn::scalar(0.0001));
    ASSERT_LE(std::abs(center.y - position.y * scale), edyn::scalar(0.0001));
    ASSERT_LE(std::abs(center.z - position.z * scale), edyn::scalar(0.0001));
}
