#include "../common/common.hpp"
#include "edyn/collision/collision_result.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/shapes/convex_mesh.hpp"
#include "edyn/shapes/cylinder_shape.hpp"
#include "edyn/shapes/polyhedron_shape.hpp"
#include "edyn/util/shape_util.hpp"
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
    auto result = edyn::collision_result{};
    edyn::collide(box, box, ctx, result);
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
    auto result = edyn::collision_result{};
    edyn::collide(box, box, ctx, result);
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

    edyn::make_box_mesh({0.5, 0.5, 0.5}, mesh->vertices, mesh->indices, mesh->faces);
    mesh->initialize();

    auto rotated = edyn::make_rotated_mesh(*mesh);

    auto polyhedron = edyn::polyhedron_shape{};
    polyhedron.mesh = mesh;
    polyhedron.rotated = &rotated;

    auto sphere = edyn::sphere_shape{0.5};

    auto ctx = edyn::collision_context{};
    ctx.posA = edyn::vector3{0.5, 0.5, 0.5};
    ctx.ornA = edyn::quaternion_identity;
    ctx.posB = edyn::vector3{0.5, 1.4, 0.5};
    ctx.ornB = edyn::quaternion_identity;
    ctx.threshold = edyn::large_scalar;

    auto result = edyn::collision_result{};
    edyn::collide(polyhedron, sphere, ctx, result);
    auto pt = result.point[0];

    ASSERT_EQ(result.num_points, 1);
    ASSERT_SCALAR_EQ(pt.normal.x, 0);
    ASSERT_SCALAR_EQ(pt.normal.y, -1);
    ASSERT_SCALAR_EQ(pt.normal.z, 0);
    ASSERT_SCALAR_EQ(pt.pivotA.x, 0);
    ASSERT_SCALAR_EQ(pt.pivotA.y, 0.5);
    ASSERT_SCALAR_EQ(pt.pivotA.z, 0);
    ASSERT_SCALAR_EQ(pt.pivotB.x, 0);
    ASSERT_SCALAR_EQ(pt.pivotB.y, -0.5);
    ASSERT_SCALAR_EQ(pt.pivotB.z, 0);
    ASSERT_SCALAR_EQ(pt.distance, -0.1);

    ctx.posA = edyn::vector3{1.5, 1.5, 0.5};
    ctx.posB = edyn::vector3{0.5, 0.5, 0.5};

    result = {};
    edyn::collide(sphere, polyhedron, ctx, result);
    pt = result.point[0];

    ASSERT_EQ(result.num_points, 1);
    ASSERT_SCALAR_EQ(pt.normal.x, 0.707107);
    ASSERT_SCALAR_EQ(pt.normal.y, 0.707107);
    ASSERT_SCALAR_EQ(pt.normal.z, 0);
    ASSERT_SCALAR_EQ(pt.pivotA.x, -0.707107/2);
    ASSERT_SCALAR_EQ(pt.pivotA.y, -0.707107/2);
    ASSERT_SCALAR_EQ(pt.pivotA.z, 0);
    ASSERT_SCALAR_EQ(pt.pivotB.x, 0.5);
    ASSERT_SCALAR_EQ(pt.pivotB.y, 0.5);
    ASSERT_SCALAR_EQ(pt.pivotB.z, 0);
    ASSERT_SCALAR_EQ(pt.distance, 0.2071067812);
}

TEST(test_collision, collide_capsule_cylinder_parallel) {
    auto capsule = edyn::capsule_shape{0.1, 0.2};
    auto cylinder = edyn::cylinder_shape{0.2, 0.5};

    auto ctx = edyn::collision_context{};
    ctx.posA = edyn::vector3{1, 0.5, 0};
    ctx.ornA = edyn::quaternion_axis_angle({0, 0, 1}, edyn::pi / 2);
    ctx.posB = edyn::vector3{0, 0, 0};
    ctx.ornB = edyn::quaternion_axis_angle({0, 0, 1}, edyn::pi / 2);
    ctx.threshold = 9999;

    auto result = edyn::collision_result{};
    edyn::collide(capsule, cylinder, ctx, result);
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

TEST(test_collision, collide_cylinder_cylinder_cap_edges) {
    auto cylinder = edyn::cylinder_shape{0.2, 0.2};

    auto ctx = edyn::collision_context{};
    ctx.posA = edyn::vector3{0.22, 0.8, -0.3};
    ctx.ornA = edyn::quaternion_axis_angle(edyn::normalize(edyn::vector3{0, 1, 0}), -edyn::pi * 0.5);
    ctx.posB = edyn::vector3{0, 0.5, 0};
    ctx.ornB = edyn::quaternion_axis_angle(edyn::normalize(edyn::vector3{0, 0, 1}), edyn::pi/2);
    ctx.threshold = 0.04;

    auto result = edyn::collision_result{};
    edyn::collide(cylinder, cylinder, ctx, result);
    ASSERT_EQ(result.num_points, 1);

    auto pivotA = edyn::vector3{0.200000003, -0.127483726, 0.154103532};
    auto pivotB = edyn::vector3{0.200000048, -0.0991190373, -0.173710704};
    auto normal = edyn::vector3{0.458552599, 0.379342318, -0.803634762};
    auto distance = edyn::scalar{-0.0848965346};
    auto &pt = result.point[0];

    ASSERT_VECTOR3_EQ(pt.pivotA, pivotA);
    ASSERT_VECTOR3_EQ(pt.pivotB, pivotB);
    ASSERT_VECTOR3_EQ(pt.normal, normal);
    ASSERT_SCALAR_EQ(pt.distance, distance);
}

TEST(test_collision, collide_cylinder_cylinder_side_edge_vs_face) {
    auto cylinder = edyn::cylinder_shape{0.2, 0.2};

    auto ctx = edyn::collision_context{};
    ctx.posA = edyn::vector3{0.1, 0.9, -0.05};
    ctx.ornA = edyn::quaternion_axis_angle(edyn::normalize(edyn::vector3{0, 1, 0}), -edyn::pi * 0.5);
    ctx.posB = edyn::vector3{0, 0.5, 0};
    ctx.ornB = edyn::quaternion_axis_angle(edyn::normalize(edyn::vector3{0, 0, 1}), 3 * edyn::pi / 2);
    ctx.threshold = 0.04;

    auto result = edyn::collision_result{};
    edyn::collide(cylinder, cylinder, ctx, result);
    ASSERT_EQ(result.num_points, 2);

    ASSERT_TRUE(std::abs(result.point[0].distance) < EDYN_EPSILON);
    ASSERT_TRUE(std::abs(result.point[1].distance) < EDYN_EPSILON);

    {
        auto pivotA = edyn::vector3{0.200000003, -0.199999973, 0};
        auto pivotB = edyn::vector3{-0.200000003, 0.100000009, 0.149999991};
        auto normal = edyn::vector3{0, 0.99999988, 0};

        ASSERT_TRUE(edyn::distance_sqr(result.point[0].pivotA, pivotA) < EDYN_EPSILON ||
                    edyn::distance_sqr(result.point[1].pivotA, pivotA) < EDYN_EPSILON);

        ASSERT_TRUE(edyn::distance_sqr(result.point[0].pivotB, pivotB) < EDYN_EPSILON ||
                    edyn::distance_sqr(result.point[1].pivotB, pivotB) < EDYN_EPSILON);

        ASSERT_TRUE(edyn::distance_sqr(result.point[0].normal, normal) < EDYN_EPSILON ||
                    edyn::distance_sqr(result.point[1].normal, normal) < EDYN_EPSILON);
    }

    {
        auto pivotA = edyn::vector3{-0.123205088, -0.199999973, 0};
        auto pivotB = edyn::vector3{-0.200000003, 0.100000009, -0.173205063};
        auto normal = edyn::vector3{0, 0.99999988, 0};

        ASSERT_TRUE(edyn::distance_sqr(result.point[0].pivotA, pivotA) < EDYN_EPSILON ||
                    edyn::distance_sqr(result.point[1].pivotA, pivotA) < EDYN_EPSILON);

        ASSERT_TRUE(edyn::distance_sqr(result.point[0].pivotB, pivotB) < EDYN_EPSILON ||
                    edyn::distance_sqr(result.point[1].pivotB, pivotB) < EDYN_EPSILON);

        ASSERT_TRUE(edyn::distance_sqr(result.point[0].normal, normal) < EDYN_EPSILON ||
                    edyn::distance_sqr(result.point[1].normal, normal) < EDYN_EPSILON);
    }
}

TEST(test_collision, collide_cylinder_cylinder_side_edge_vs_side_edge_parallel) {
    auto cylinder = edyn::cylinder_shape{0.2, 0.2};

    auto ctx = edyn::collision_context{};
    ctx.posA = edyn::vector3{1, 0.6, 0};
    ctx.ornA = edyn::quaternion_axis_angle({0, 0, 1}, edyn::pi * 0.5);
    ctx.posB = edyn::vector3{0.6, 0.6, 0};
    ctx.ornB = edyn::quaternion_axis_angle({0, 0, 1}, edyn::pi * 0.5);
    ctx.threshold = 0.04;

    auto result = edyn::collision_result{};
    edyn::collide(cylinder, cylinder, ctx, result);
    ASSERT_EQ(result.num_points, 2);

    ASSERT_TRUE(std::abs(result.point[0].distance) < EDYN_EPSILON);
    ASSERT_TRUE(std::abs(result.point[1].distance) < EDYN_EPSILON);

    {
        auto pivotA = edyn::vector3{0.199999988, 0.199999988, 0};
        auto pivotB = edyn::vector3{0.199999988, -0.199999988, 0};
        auto normal = edyn::vector3{0.99999988, 0, 0};

        ASSERT_TRUE(edyn::distance_sqr(result.point[0].pivotA, pivotA) < EDYN_EPSILON ||
                    edyn::distance_sqr(result.point[1].pivotA, pivotA) < EDYN_EPSILON);

        ASSERT_TRUE(edyn::distance_sqr(result.point[0].pivotB, pivotB) < EDYN_EPSILON ||
                    edyn::distance_sqr(result.point[1].pivotB, pivotB) < EDYN_EPSILON);

        ASSERT_TRUE(edyn::distance_sqr(result.point[0].normal, normal) < EDYN_EPSILON ||
                    edyn::distance_sqr(result.point[1].normal, normal) < EDYN_EPSILON);
    }

    {
        auto pivotA = edyn::vector3{-0.199999988, 0.199999988, 0};
        auto pivotB = edyn::vector3{-0.199999988, -0.199999988, 0};
        auto normal = edyn::vector3{0.99999988, 0, 0};

        ASSERT_TRUE(edyn::distance_sqr(result.point[0].pivotA, pivotA) < EDYN_EPSILON ||
                    edyn::distance_sqr(result.point[1].pivotA, pivotA) < EDYN_EPSILON);

        ASSERT_TRUE(edyn::distance_sqr(result.point[0].pivotB, pivotB) < EDYN_EPSILON ||
                    edyn::distance_sqr(result.point[1].pivotB, pivotB) < EDYN_EPSILON);

        ASSERT_TRUE(edyn::distance_sqr(result.point[0].normal, normal) < EDYN_EPSILON ||
                    edyn::distance_sqr(result.point[1].normal, normal) < EDYN_EPSILON);
    }
}

TEST(test_collision, collide_cylinder_box_side_edge_vs_edge_parallel) {
    auto cylinder = edyn::cylinder_shape{0.2, 0.2};
    auto box = edyn::box_shape{0.3, 0.1, 0.2};

    auto ctx = edyn::collision_context{};
    ctx.posA = {0.12, 0.9, -0.05};
    ctx.ornA = edyn::quaternion_axis_angle(edyn::normalize(edyn::vector3{0, 1, 0}), -edyn::pi * 0.5);
    ctx.posB = {0, 0.5, 0};
    ctx.ornB = edyn::quaternion_axis_angle(edyn::normalize(edyn::vector3{0, 0, 1}), 3 * edyn::pi/2);
    ctx.threshold = 0.04;

    auto result = edyn::collision_result{};
    edyn::collide(cylinder, box, ctx, result);
    ASSERT_EQ(result.num_points, 2);

    auto expected_distance = edyn::scalar(-0.0980195999);
    ASSERT_SCALAR_EQ(result.point[0].distance, expected_distance);
    ASSERT_SCALAR_EQ(result.point[1].distance, expected_distance);

    {
        auto pivotA = edyn::vector3{0.199999988, -0.196116149, 0.0392232165};
        auto pivotB = edyn::vector3{-0.300000012, 0.100000009, 0.149999991};
        auto normal = edyn::vector3{0.19611609, 0.980580687, 0};

        ASSERT_TRUE(edyn::distance_sqr(result.point[0].pivotA, pivotA) < EDYN_EPSILON ||
                    edyn::distance_sqr(result.point[1].pivotA, pivotA) < EDYN_EPSILON);

        ASSERT_TRUE(edyn::distance_sqr(result.point[0].pivotB, pivotB) < EDYN_EPSILON ||
                    edyn::distance_sqr(result.point[1].pivotB, pivotB) < EDYN_EPSILON);

        ASSERT_TRUE(edyn::distance_sqr(result.point[0].normal, normal) < EDYN_EPSILON ||
                    edyn::distance_sqr(result.point[1].normal, normal) < EDYN_EPSILON);
    }

    {
        auto pivotA = edyn::vector3{-0.149999991, -0.196116149, 0.039223209};
        auto pivotB = edyn::vector3{-0.300000012, 0.100000009, -0.200000003};
        auto normal = edyn::vector3{0.19611609, 0.980580687, 0};

        ASSERT_TRUE(edyn::distance_sqr(result.point[0].pivotA, pivotA) < EDYN_EPSILON ||
                    edyn::distance_sqr(result.point[1].pivotA, pivotA) < EDYN_EPSILON);

        ASSERT_TRUE(edyn::distance_sqr(result.point[0].pivotB, pivotB) < EDYN_EPSILON ||
                    edyn::distance_sqr(result.point[1].pivotB, pivotB) < EDYN_EPSILON);

        ASSERT_TRUE(edyn::distance_sqr(result.point[0].normal, normal) < EDYN_EPSILON ||
                    edyn::distance_sqr(result.point[1].normal, normal) < EDYN_EPSILON);
    }
}

TEST(test_collision, collide_cylinder_box_side_edge_vs_edge) {
    auto cylinder = edyn::cylinder_shape{0.2, 0.2};
    auto box = edyn::box_shape{0.3, 0.1, 0.2};

    auto ctx = edyn::collision_context{};
    ctx.posA = {0.13, 1, 0.05};
    ctx.ornA = edyn::quaternion_axis_angle(edyn::normalize(edyn::vector3{0, 0, 1}), -edyn::pi * 0.25);
    ctx.posB = {0, 0.5, 0};
    ctx.ornB = edyn::quaternion_axis_angle(edyn::normalize(edyn::vector3{0, 0, 1}), 3 * edyn::pi/2);
    ctx.threshold = 0.04;

    auto result = edyn::collision_result{};
    edyn::collide(cylinder, box, ctx, result);
    ASSERT_EQ(result.num_points, 1);

    auto &pt = result.point[0];

    ASSERT_SCALAR_EQ(result.point[0].distance, edyn::scalar(-0.0373654366));

    auto pivotA = edyn::vector3{0.120208159, -0.199999988, 0};
    auto pivotB = edyn::vector3{-0.300000012, 0.100000009, 0.049999997};
    auto normal = edyn::vector3{0.707106649, 0.707106769, 0};

    ASSERT_TRUE(edyn::distance_sqr(pt.pivotA, pivotA) < EDYN_EPSILON);
    ASSERT_TRUE(edyn::distance_sqr(pt.pivotB, pivotB) < EDYN_EPSILON);
    ASSERT_TRUE(edyn::distance_sqr(pt.normal, normal) < EDYN_EPSILON);
}

TEST(test_collision, collide_cylinder_box_cap_edge_vs_edge) {
    auto cylinder = edyn::cylinder_shape{0.2, 0.2};
    auto box = edyn::box_shape{0.3, 0.1, 0.2};

    auto ctx = edyn::collision_context{};
    ctx.posA = {0.1, 0.9, 0.01};
    ctx.ornA = edyn::quaternion_axis_angle(edyn::normalize(edyn::vector3{0, 0, 1}), -edyn::pi * 0.25);
    ctx.posB = {0, 0.5, 0};
    ctx.ornB = edyn::quaternion_axis_angle(edyn::normalize(edyn::vector3{1, 0, 0}), edyn::pi/3);
    ctx.threshold = 0.04;

    auto result = edyn::collision_result{};
    edyn::collide(cylinder, box, ctx, result);
    ASSERT_EQ(result.num_points, 1);

    auto &pt = result.point[0];

    ASSERT_SCALAR_EQ(result.point[0].distance, edyn::scalar(-0.104495525));

    auto pivotA = edyn::vector3{0.199999988, -0.196491167, -0.0372988172};
    auto pivotB = edyn::vector3{0.102481082, 0.099999994, -0.200000018};
    auto normal = edyn::rotate(ctx.ornB, {0, 0.610765815, -0.791811347});

    ASSERT_TRUE(edyn::distance_sqr(pt.pivotA, pivotA) < EDYN_EPSILON);
    ASSERT_TRUE(edyn::distance_sqr(pt.pivotB, pivotB) < EDYN_EPSILON);
    ASSERT_TRUE(edyn::distance_sqr(pt.normal, normal) < EDYN_EPSILON);
}

TEST(test_collision, collide_cylinder_box_face_face) {
    auto cylinder = edyn::cylinder_shape{0.2, 0.2};
    auto box = edyn::box_shape{0.3, 0.1, 0.2};

    auto ctx = edyn::collision_context{};
    ctx.posA = {0.21, 0.8, 0.02};
    ctx.ornA = edyn::quaternion_axis_angle(edyn::normalize(edyn::vector3{0, 0, 1}), -edyn::pi * 0.5);
    ctx.posB = {0, 0.5, 0};
    ctx.ornB = edyn::quaternion_axis_angle(edyn::normalize(edyn::vector3{0, 1, 0}), edyn::pi/2);
    ctx.threshold = 0.04;

    auto result = edyn::collision_result{};
    edyn::collide(cylinder, box, ctx, result);
    ASSERT_EQ(result.num_points, 3);

    for (size_t i = 0; i < result.num_points; ++i) {
        ASSERT_TRUE(std::abs(result.point[i].distance) < EDYN_EPSILON);
        ASSERT_TRUE(edyn::distance_sqr(result.point[i].normal, edyn::vector3_y) < EDYN_EPSILON);
    }

    {
        auto pivotA = edyn::vector3{0.2, -0.01, 0.2};
        auto pivotB = edyn::vector3{-0.22, 0.1, 0.2};
        bool containsA = false;
        bool containsB = false;

        for (size_t i = 0; i < result.num_points; ++i) {
            containsA |= edyn::distance_sqr(result.point[i].pivotA, pivotA) < EDYN_EPSILON;
            containsB |= edyn::distance_sqr(result.point[i].pivotB, pivotB) < EDYN_EPSILON;
        }

        ASSERT_TRUE(containsA);
        ASSERT_TRUE(containsB);
    }

    {
        auto pivotA = edyn::vector3{0.2, -0.01, -0.2};
        auto pivotB = edyn::vector3{0.18, 0.1, 0.2};
        bool containsA = false;
        bool containsB = false;

        for (size_t i = 0; i < result.num_points; ++i) {
            containsA |= edyn::distance_sqr(result.point[i].pivotA, pivotA) < EDYN_EPSILON;
            containsB |= edyn::distance_sqr(result.point[i].pivotB, pivotB) < EDYN_EPSILON;
        }

        ASSERT_TRUE(containsA);
        ASSERT_TRUE(containsB);
    }

    {
        auto pivotA = edyn::vector3{0.2, -0.2, 0};
        auto pivotB = edyn::vector3{-0.02, 0.1, 0.01};
        bool containsA = false;
        bool containsB = false;

        for (size_t i = 0; i < result.num_points; ++i) {
            containsA |= edyn::distance_sqr(result.point[i].pivotA, pivotA) < EDYN_EPSILON;
            containsB |= edyn::distance_sqr(result.point[i].pivotB, pivotB) < EDYN_EPSILON;
        }

        ASSERT_TRUE(containsA);
        ASSERT_TRUE(containsB);
    }
}