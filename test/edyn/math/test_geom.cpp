#include "../common/common.hpp"

TEST(geom_test, intersect_line_aabb) {
    auto p0 = edyn::vector2{0, 0.5};
    auto p1 = edyn::vector2{1, 1.5};
    auto aabb_min = edyn::vector2{-1, -0.5};
    auto aabb_max = edyn::vector2{2, 1};
    edyn::scalar s[2];
    auto num_points = edyn::intersect_line_aabb(p0, p1, aabb_min, aabb_max, s[0], s[1]);
    ASSERT_EQ(num_points, 2);
    ASSERT_SCALAR_EQ(s[0], -1);
    ASSERT_SCALAR_EQ(s[1], 0.5);
}

TEST(geom_test, intersect_line_aabb_no_intersection) {
    auto p0 = edyn::vector2{0, 0};
    auto p1 = edyn::vector2{1, 1.5};
    auto aabb_min = edyn::vector2{-1, 0.5};
    auto aabb_max = edyn::vector2{0, 1};
    edyn::scalar s[2];
    auto num_points = edyn::intersect_line_aabb(p0, p1, aabb_min, aabb_max, s[0], s[1]);
    ASSERT_EQ(num_points, 0);
}

TEST(geom_test, intersect_line_aabb_parallel) {
    auto p0 = edyn::vector2{1, 1};
    auto p1 = edyn::vector2{1, -1};
    auto aabb_min = edyn::vector2{-2, -0.5};
    auto aabb_max = edyn::vector2{1, 0.5};
    edyn::scalar s[2];
    auto num_points = edyn::intersect_line_aabb(p0, p1, aabb_min, aabb_max, s[0], s[1]);
    ASSERT_EQ(num_points, 2);
    ASSERT_SCALAR_EQ(s[0], 0.75);
    ASSERT_SCALAR_EQ(s[1], 0.25);
}

TEST(geom_test, intersect_line_aabb_parallel_2) {
    auto p0 = edyn::vector2{0, -0.25};
    auto p1 = edyn::vector2{1, -0.25};
    auto aabb_min = edyn::vector2{-2, -0.5};
    auto aabb_max = edyn::vector2{1, 0.5};
    edyn::scalar s[2];
    auto num_points = edyn::intersect_line_aabb(p0, p1, aabb_min, aabb_max, s[0], s[1]);
    ASSERT_EQ(num_points, 2);
    ASSERT_SCALAR_EQ(s[0], -2);
    ASSERT_SCALAR_EQ(s[1], 1);
}