#include "../common/common.hpp"
#include <edyn/collision/collide.hpp>

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