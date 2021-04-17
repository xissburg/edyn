#include "../common/common.hpp"
#include "edyn/comp/rotated_mesh.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/scalar.hpp"
#include "edyn/shapes/convex_mesh.hpp"
#include "edyn/shapes/cylinder_shape.hpp"
#include "edyn/shapes/polyhedron_shape.hpp"
#include <edyn/collision/collide.hpp>
#include <memory>

TEST(test_collision, collide_box_box_face_face) {
    auto box = edyn::box_shape{edyn::vector3{0.5, 0.5, 0.5}};
    auto ctx = edyn::collision_context{};
    ctx.posA = edyn::vector3{0,0,0};
    ctx.ornA = edyn::quaternion_identity;
    ctx.posB = edyn::vector3{0, 2 * box.half_extents.y, 0};
    ctx.ornB = edyn::quaternion_identity;
    ctx.threshold = 0.02;
    auto result = edyn::collide(box, box, ctx);
    ASSERT_EQ(result.num_points, 4);
    
    std::vector<edyn::vector3> expected_points;
    expected_points.push_back(edyn::vector3{0.5, 0.5, 0.5});
    expected_points.push_back(edyn::vector3{-0.5, 0.5, 0.5});
    expected_points.push_back(edyn::vector3{-0.5, 0.5, -0.5});
    expected_points.push_back(edyn::vector3{0.5, 0.5, -0.5});

    for (size_t i = 0; i < 4; ++i) {
        ASSERT_EQ(expected_points.size(), 4 - i);
        
        for (auto it = expected_points.begin(); it != expected_points.end(); ++it) {
            if (edyn::distance(result.point[i].pivotA, *it) < EDYN_EPSILON) {
                expected_points.erase(it);
                break;
            }
        }
    }

    ASSERT_TRUE(expected_points.empty());
}

TEST(test_collision, collide_box_box_face_edge) {
    auto box = edyn::box_shape{edyn::vector3{0.5, 0.5, 0.5}};
    auto ctx = edyn::collision_context{};
    ctx.posA = edyn::vector3{0,0,0};
    ctx.ornA = edyn::quaternion_identity;
    ctx.posB = edyn::vector3{0, edyn::scalar{2} * box.half_extents.y + edyn::scalar{0.2}, 0};
    ctx.ornB = edyn::quaternion_axis_angle({1, 0, 0}, edyn::pi / 4);
    ctx.threshold = 0.02;
    auto result = edyn::collide(box, box, ctx);
    ASSERT_EQ(result.num_points, 2);
    
    std::vector<edyn::vector3> expected_pivotA;
    expected_pivotA.push_back(edyn::vector3{0.5, 0.5, 0});
    expected_pivotA.push_back(edyn::vector3{-0.5, 0.5, 0});

    for (size_t i = 0; i < 2; ++i) {
        ASSERT_EQ(expected_pivotA.size(), 2 - i);
        
        for (auto it = expected_pivotA.begin(); it != expected_pivotA.end(); ++it) {
            if (edyn::distance(result.point[i].pivotA, *it) < EDYN_EPSILON) {
                expected_pivotA.erase(it);
                break;
            }
        }
    }

    ASSERT_TRUE(expected_pivotA.empty());
    
    std::vector<edyn::vector3> expected_pivotB;
    expected_pivotB.push_back(edyn::vector3{0.5, -0.5, 0.5});
    expected_pivotB.push_back(edyn::vector3{-0.5, -0.5, 0.5});

    for (size_t i = 0; i < 2; ++i) {
        ASSERT_EQ(expected_pivotB.size(), 2 - i);
        
        for (auto it = expected_pivotB.begin(); it != expected_pivotB.end(); ++it) {
            if (edyn::distance(result.point[i].pivotB, *it) < EDYN_EPSILON) {
                expected_pivotB.erase(it);
                break;
            }
        }
    }

    ASSERT_TRUE(expected_pivotB.empty());
}

TEST(test_collision, collide_polyhedron_sphere) {
    auto mesh = std::make_shared<edyn::convex_mesh>();
    // Make a box.
    mesh->vertices.push_back({0, 0, 0});
    mesh->vertices.push_back({1, 0, 0});
    mesh->vertices.push_back({1, 0, 1});
    mesh->vertices.push_back({0, 0, 1});
    mesh->vertices.push_back({0, 1, 0});
    mesh->vertices.push_back({1, 1, 0});
    mesh->vertices.push_back({1, 1, 1});
    mesh->vertices.push_back({0, 1, 1});

    mesh->indices.insert(mesh->indices.end(), {0, 1, 2, 3}); // bottom
    mesh->indices.insert(mesh->indices.end(), {7, 6, 5, 4}); // top
    mesh->indices.insert(mesh->indices.end(), {4, 5, 1, 0}); // front
    mesh->indices.insert(mesh->indices.end(), {6, 7, 3, 2}); // rear
    mesh->indices.insert(mesh->indices.end(), {7, 4, 0, 3}); // left
    mesh->indices.insert(mesh->indices.end(), {5, 6, 2, 1}); // right

    mesh->faces.insert(mesh->faces.end(), {0, 4});
    mesh->faces.insert(mesh->faces.end(), {4, 4});
    mesh->faces.insert(mesh->faces.end(), {8, 4});
    mesh->faces.insert(mesh->faces.end(), {12, 4});
    mesh->faces.insert(mesh->faces.end(), {16, 4});
    mesh->faces.insert(mesh->faces.end(), {20, 4});
    
    mesh->calculate_normals();
    mesh->calculate_edges();

    auto polyhedron = edyn::polyhedron_shape{};
    polyhedron.mesh = mesh;

    auto rmesh = edyn::rotated_mesh{};
    rmesh.vertices = mesh->vertices;
    rmesh.normals = mesh->normals;

    auto sphere = edyn::sphere_shape{0.5};

    auto ctx = edyn::collision_context{};
    ctx.posA = edyn::vector3{0, 0, 0};
    ctx.ornA = edyn::quaternion_identity;
    ctx.posB = edyn::vector3{0.5, 1.4, 0.5};
    ctx.ornB = edyn::quaternion_identity;
    ctx.rmeshA = rmesh; ctx.rmeshB = rmesh;
    ctx.threshold = edyn::large_scalar;
    auto result = edyn::collide(polyhedron, sphere, ctx);
    auto pt = result.point[0];
    
    ASSERT_EQ(result.num_points, 1);
    ASSERT_SCALAR_EQ(pt.normalB.x, 0);
    ASSERT_SCALAR_EQ(pt.normalB.y, -1);
    ASSERT_SCALAR_EQ(pt.normalB.z, 0);
    ASSERT_SCALAR_EQ(pt.pivotA.x, 0.5);
    ASSERT_SCALAR_EQ(pt.pivotA.y, 1);
    ASSERT_SCALAR_EQ(pt.pivotA.z, 0.5);
    ASSERT_SCALAR_EQ(pt.pivotB.x, 0);
    ASSERT_SCALAR_EQ(pt.pivotB.y, -0.5);
    ASSERT_SCALAR_EQ(pt.pivotB.z, 0);
    ASSERT_SCALAR_EQ(pt.distance, -0.1);

    ctx.posA = edyn::vector3{1.5, 1.5, 0.5};
    ctx.posB = edyn::vector3{0, 0, 0};
    result = edyn::collide(sphere, polyhedron, ctx);
    pt = result.point[0];

    ASSERT_EQ(result.num_points, 1);
    ASSERT_SCALAR_EQ(pt.normalB.x, 0.707107);
    ASSERT_SCALAR_EQ(pt.normalB.y, 0.707107);
    ASSERT_SCALAR_EQ(pt.normalB.z, 0);
    ASSERT_SCALAR_EQ(pt.pivotA.x, -0.707107/2);
    ASSERT_SCALAR_EQ(pt.pivotA.y, -0.707107/2);
    ASSERT_SCALAR_EQ(pt.pivotA.z, 0);
    ASSERT_SCALAR_EQ(pt.pivotB.x, 1);
    ASSERT_SCALAR_EQ(pt.pivotB.y, 1);
    ASSERT_SCALAR_EQ(pt.pivotB.z, 0.5);
    ASSERT_SCALAR_EQ(pt.distance, 0.2071067812);
}

TEST(test_collision, collide_capsule_cylinder_parallel) {
    auto capsule = edyn::capsule_shape{0.1, 0.2};
    auto cylinder = edyn::cylinder_shape{0.2, 0.5};

    auto ctx = edyn::collision_context{};
    ctx.posA = edyn::vector3{0.3, 0.5, 0};
    ctx.ornA = edyn::quaternion_axis_angle({0, 0, 1}, edyn::pi / 2);
    ctx.posB = edyn::vector3{0, 0, 0};
    ctx.ornB = edyn::quaternion_axis_angle({0, 0, 1}, edyn::pi / 2);
    ctx.threshold = 0.02;
    auto result = edyn::collide(capsule, cylinder, ctx);
    ASSERT_EQ(result.num_points, 2);

    std::vector<edyn::vector3> expected_pivotA;
    expected_pivotA.push_back(edyn::vector3{-0.2, 0.1, 0});
    expected_pivotA.push_back(edyn::vector3{0.0, 0.1, 0});

    for (size_t i = 0; i < 2; ++i) {
        ASSERT_EQ(expected_pivotA.size(), 2 - i);
        
        for (auto it = expected_pivotA.begin(); it != expected_pivotA.end(); ++it) {
            if (edyn::distance(result.point[i].pivotA, *it) < EDYN_EPSILON) {
                expected_pivotA.erase(it);
                break;
            }
        }
    }

    ASSERT_TRUE(expected_pivotA.empty());
    
    std::vector<edyn::vector3> expected_pivotB;
    expected_pivotB.push_back(edyn::vector3{0.5, -0.2, 0});
    expected_pivotB.push_back(edyn::vector3{0.3, -0.2, 0});

    for (size_t i = 0; i < 2; ++i) {
        ASSERT_EQ(expected_pivotB.size(), 2 - i);
        
        for (auto it = expected_pivotB.begin(); it != expected_pivotB.end(); ++it) {
            if (edyn::distance(result.point[i].pivotB, *it) < EDYN_EPSILON) {
                expected_pivotB.erase(it);
                break;
            }
        }
    }

    ASSERT_TRUE(expected_pivotB.empty());
}