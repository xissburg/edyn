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

TEST(geom_test, closest_point_circle_line_parallel) {
    auto cpos = edyn::vector3{1, 0, 0};
    auto corn = edyn::quaternion_axis_angle({0, 1, 0}, 2);
    auto radius = 2;
    auto axis = edyn::coordinate_axis::y;
    auto p0 = edyn::vector3{0.5, 1, -2};
    auto p1 = edyn::vector3{0.5, 1, 2};
    size_t n;
    edyn::scalar s0, s1;
    edyn::vector3 rc0, rl0, rc1, rl1, normal;

    auto dist_sqr = edyn::closest_point_circle_line(cpos, corn, radius, axis, p0, p1, n, s0, rc0, rl0, s1, rc1, rl1, normal);

    ASSERT_SCALAR_EQ(dist_sqr, 1);
    ASSERT_EQ(n, 2);
    ASSERT_LT(std::abs(s0 - (2 - std::cos(std::atan(0.5/2)) * radius) / (p1.z - p0.z)), edyn::scalar(0.001));
    ASSERT_SCALAR_EQ(s1, 1 - s0);
    ASSERT_SCALAR_EQ(normal.x, 0);
    ASSERT_SCALAR_EQ(normal.y, -1);
    ASSERT_SCALAR_EQ(normal.z, 0);
}

TEST(geom_test, closest_point_circle_line_diagonal) {
    auto cpos = edyn::vector3{0, 0, 0};
    auto corn = edyn::quaternion_identity;
    auto radius = 2;
    auto axis = edyn::coordinate_axis::y;
    auto p0 = edyn::vector3{5, 1, 0};
    auto p1 = edyn::vector3{0, 1, 5};
    size_t n;
    edyn::scalar s0, s1;
    edyn::vector3 rc0, rl0, rc1, rl1, normal;

    auto dist_sqr = edyn::closest_point_circle_line(cpos, corn, radius, axis, p0, p1, n, s0, rc0, rl0, s1, rc1, rl1, normal);

    ASSERT_SCALAR_EQ(dist_sqr, 3.3578644);
    ASSERT_EQ(n, 1);
    ASSERT_EQ(s0, 0.5);
    ASSERT_SCALAR_EQ(normal.x, 0.59253341);
    ASSERT_SCALAR_EQ(normal.y, 0.54571818);
    ASSERT_SCALAR_EQ(normal.z, 0.59253341);
}

TEST(geom_test, closest_point_circle_circle_parallel) {
    auto posA = edyn::vector3{0, 0, 0};
    auto ornA = edyn::quaternion_identity;
    auto radiusA = edyn::scalar(2);
    auto axisA = edyn::coordinate_axis::z;
    auto posB = edyn::vector3{0.5, 0.5, 0};
    auto ornB = edyn::quaternion_identity;
    auto radiusB = edyn::scalar(1);
    auto axisB = edyn::coordinate_axis::z;
    size_t n;
    edyn::vector3 rA0, rB0, rA1, rB1, normal;

    auto dist_sqr = edyn::closest_point_circle_circle(posA, ornA, radiusA, axisA,
                                                      posB, ornB, radiusB, axisB,
                                                      n, rA0, rB0, rA1, rB1, normal);

    ASSERT_EQ(n, 1);
    ASSERT_SCALAR_EQ(rA0.x, 1.41421356);
    ASSERT_SCALAR_EQ(rA0.y, 1.41421356);
    ASSERT_SCALAR_EQ(rA0.z, 0);
    ASSERT_SCALAR_EQ(rB0.x, 1.20710678);
    ASSERT_SCALAR_EQ(rB0.y, 1.20710678);
    ASSERT_SCALAR_EQ(rB0.z, 0);
    ASSERT_SCALAR_EQ(dist_sqr, 0.08578638);
    ASSERT_SCALAR_EQ(normal.x, 0);
    ASSERT_SCALAR_EQ(normal.y, 0);
    ASSERT_SCALAR_EQ(normal.z, 1);
}
