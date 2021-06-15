#include "../common/common.hpp"
#include "edyn/math/geom.hpp"

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

TEST(geom_test, intersect_line_aabb_one_intersection) {
    auto p0 = edyn::vector2{2, 1};
    auto p1 = edyn::vector2{1, 1.5};
    auto aabb_min = edyn::vector2{-1, -0.5};
    auto aabb_max = edyn::vector2{2, 1};
    edyn::scalar s[2];
    auto num_points = edyn::intersect_line_aabb(p0, p1, aabb_min, aabb_max, s[0], s[1]);
    ASSERT_EQ(num_points, 1);
    ASSERT_SCALAR_EQ(s[0], 0);
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

TEST(geom_test, intersect_segments_parallel) {
    auto p0 = edyn::vector2{-1, -1};
    auto p1 = edyn::vector2{2, 2};
    auto q0 = edyn::vector2{0, 0};
    auto q1 = edyn::vector2{3, 3};
    edyn::scalar s[2], t[2];
    auto num_points = edyn::intersect_segments(p0, p1, q0, q1, s[0], t[0], s[1], t[1]);
    ASSERT_EQ(num_points, 2);
    ASSERT_SCALAR_EQ(s[0], edyn::scalar(1.0 / 3.0));
    ASSERT_SCALAR_EQ(s[1], edyn::scalar(1));
    ASSERT_SCALAR_EQ(t[0], edyn::scalar(0));
    ASSERT_SCALAR_EQ(t[1], edyn::scalar(2.0 / 3.0));
}

TEST(geom_test, intersect_segments_parallel_reversed) {
    auto p0 = edyn::vector2{-1, -1};
    auto p1 = edyn::vector2{2, 2};
    auto q0 = edyn::vector2{3, 3};
    auto q1 = edyn::vector2{0, 0};
    edyn::scalar s[2], t[2];
    auto num_points = edyn::intersect_segments(p0, p1, q0, q1, s[0], t[0], s[1], t[1]);
    ASSERT_EQ(num_points, 2);
    ASSERT_SCALAR_EQ(s[0], edyn::scalar(1));
    ASSERT_SCALAR_EQ(s[1], edyn::scalar(1.0 / 3.0));
    ASSERT_SCALAR_EQ(t[0], edyn::scalar(1));
    ASSERT_SCALAR_EQ(t[1], edyn::scalar(1.0 / 3.0));
}

TEST(geom_test, intersect_segments_identical) {
    auto p0 = edyn::vector2{-1.5, 2.3};
    auto p1 = edyn::vector2{8.66, 0.98};
    edyn::scalar s[2], t[2];
    auto num_points = edyn::intersect_segments(p0, p1, p0, p1, s[0], t[0], s[1], t[1]);
    ASSERT_EQ(num_points, 2);
    ASSERT_SCALAR_EQ(s[0], edyn::scalar(0));
    ASSERT_SCALAR_EQ(s[1], edyn::scalar(1));
    ASSERT_SCALAR_EQ(t[0], edyn::scalar(0));
    ASSERT_SCALAR_EQ(t[1], edyn::scalar(1));
}

TEST(geom_test, intersect_segments_identical_reversed) {
    auto p0 = edyn::vector2{-1.5, 2.3};
    auto p1 = edyn::vector2{8.66, 0.98};
    edyn::scalar s[2], t[2];
    auto num_points = edyn::intersect_segments(p0, p1, p1, p0, s[0], t[0], s[1], t[1]);
    ASSERT_EQ(num_points, 2);
    ASSERT_SCALAR_EQ(s[0], edyn::scalar(1));
    ASSERT_SCALAR_EQ(s[1], edyn::scalar(0));
    ASSERT_SCALAR_EQ(t[0], edyn::scalar(1));
    ASSERT_SCALAR_EQ(t[1], edyn::scalar(0));
}

TEST(geom_test, intersect_segments_parallel_no_intersection) {
    auto p0 = edyn::vector2{-1, -1};
    auto p1 = edyn::vector2{2, 2};
    auto q1 = edyn::vector2{3, 3};
    auto q0 = edyn::vector2{8, 8};
    edyn::scalar s[2], t[2];
    auto num_points = edyn::intersect_segments(p0, p1, q0, q1, s[0], t[0], s[1], t[1]);
    ASSERT_EQ(num_points, 0);
    // reversed
    q0 = edyn::vector2{-8, -8};
    q1 = edyn::vector2{-3, -3};
    num_points = edyn::intersect_segments(p0, p1, q0, q1, s[0], t[0], s[1], t[1]);
    ASSERT_EQ(num_points, 0);
}

TEST(geom_test, intersect_segments_parallel_single) {
    auto p0 = edyn::vector2{-1, -1};
    auto p1 = edyn::vector2{2, 2};
    auto q0 = edyn::vector2{2, 2};
    auto q1 = edyn::vector2{3, 3};
    edyn::scalar s[2], t[2];
    auto num_points = edyn::intersect_segments(p0, p1, q0, q1, s[0], t[0], s[1], t[1]);
    ASSERT_EQ(num_points, 1);
    ASSERT_SCALAR_EQ(s[0], edyn::scalar(1));
    ASSERT_SCALAR_EQ(t[0], edyn::scalar(0));
}