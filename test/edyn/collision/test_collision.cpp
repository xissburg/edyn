#include "../common/common.hpp"
#include <edyn/collision/collide.hpp>

TEST(test_collision, collide_box_box) {
    auto box = edyn::box_shape{edyn::vector3{0.5, 0.5, 0.5}};
    auto result = edyn::collide(box, edyn::vector3{0,0,0}, edyn::quaternion_identity,
                                box, edyn::vector3{0, 2 * box.half_extents.y, 0}, edyn::quaternion_identity, 
                                0.02);
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