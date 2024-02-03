#ifndef EDYN_MATH_GEOM_HPP
#define EDYN_MATH_GEOM_HPP

#include <vector>
#include <cstdint>
#include "edyn/config/config.h"
#include "edyn/config/constants.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/vector2.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/coordinate_axis.hpp"
#include "edyn/util/array_util.hpp"

namespace edyn {
/**
 * Geometric utilities.
 */

/**
 * @brief Computes the point in the segment `q(t) = q0 + t*(q1 - q0)` closest
 * to point `p`.
 *
 * @param q0 Initial point in segment.
 * @param q1 End point in segment.
 * @param p The point.
 * @param t Outputs the parameter where `q(t)` gives the closest point to `p`.
 * @param q Outputs the point in `q(t)` closest to `p`.
 * @return The squared distance between `q(t)` an `p`.
 */
scalar closest_point_segment(const vector3 &q0, const vector3 &q1,
                             const vector3 &p, scalar &t, vector3 &q) noexcept;

/**
 * @brief Computes the squared distance between a point and a line.
 * @param q0 A point in the line.
 * @param dir Direction vector of line.
 * @param p The point.
 * @return Square of distance between point and line.
 */
scalar distance_sqr_line(const vector3 &q0, const vector3 &dir,
                         const vector3 &p) noexcept;

/**
 * @brief Computes the point in the line `q(t) = q0 + t*dir` closest
 * to point `p`.
 *
 * @param q0 Point in line.
 * @param dir Line direction vector.
 * @param p The point.
 * @param t Outputs the parameter where `q(t)` gives the closest point to `p`.
 * @param r Outputs the point in `q(t)` closest to `p`.
 * @return The squared distance between `q(t)` an `p`.
 */
scalar closest_point_line(const vector3 &q0, const vector3 &dir,
                          const vector3 &p, scalar &t, vector3 &r) noexcept;

/**
 * @brief Computes the parameters for the closest points of two *non-parallel*
 * lines `L1(s) = p1 + s*(q1 - p1)` and `L2(t) = p2 + t*(q2 - p2)`.
 * @param p1 A point in the first line.
 * @param q1 Another point in the first line.
 * @param p2 A point in the second line.
 * @param q2 Another point in the second line.
 * @param s Outputs the parameter where `L1(s)` gives the closest point to `L2`.
 * @param t Outputs the parameter where `L2(t)` gives the closest point to `L1`.
 * @return False if lines are parallel, which means the projection of any point
 * of L1 on L2 is a valid closest point.
 */
bool closest_point_line_line(const vector3 &p1, const vector3 &q1,
                             const vector3 &p2, const vector3 &q2,
                             scalar &s, scalar &t) noexcept;

/**
 * @brief Computes the closest points `c1` and `c2` of segments
 * `s1(s) = p1 + s*(q1 - p1)` and `s2(t) = p2 + t*(q2 - p2)`,
 * where `0 <= s <= 1` and `0 <= t <= 1`.
 *
 * @param p1 Initial point in the first segment.
 * @param q1 End point in the first segment.
 * @param p2 Initial point in the second segment.
 * @param q2 End point in the second segment.
 * @param s Outputs the parameter where `s1(s)` gives the closest point to `s2`.
 * @param t Outputs the parameter where `s2(t)` gives the closest point to `s1`.
 * @param c1 Outputs the point in `s1` closest to `s2`.
 * @param c2 Outputs the point in `s2` closest to `s1`.
 * @param num_points Optional pointer to store the number of closest points. If
 *        not `nullptr` and the segments are parallel, two closest points will
 *        be generated if the projection of one segment onto the other is a range
 *        of points.
 * @param sp Outputs the parameter where `s1(s)` gives the closest point to `s2`
 *        if segments are parallel.
 * @param tp Outputs the parameter where `s2(t)` gives the closest point to `s1`
 *        if segments are parallel.
 * @param c1p Outputs the second point in `s1` closest to `s2` if segments are
 *        parallel.
 * @param c2p Outputs the second point in `s2` closest to `s1` if segments are
 *        parallel.
 * @return The squared distance between `s1(s)` and `s2(t)`.
 */
scalar closest_point_segment_segment(const vector3 &p1, const vector3 &q1,
                                     const vector3 &p2, const vector3 &q2,
                                     scalar &s, scalar &t,
                                     vector3 &c1, vector3 &c2,
                                     size_t *num_points = nullptr,
                                     scalar *sp = nullptr, scalar *tp = nullptr,
                                     vector3 *c1p = nullptr, vector3 *c2p = nullptr) noexcept;

/**
 * Find closest point in a disc to a given point.
 * @param dpos Center of disc.
 * @param dorn Orientation of disc.
 * @param radius Radius of disc.
 * @param axis Coordinate axis orthogonal to disc plane in object space.
 * @param p Query point.
 * @param q Output for point in disc closest to `p`.
 * @return The squared distance between `p` and `q`.
 */
scalar closest_point_disc(const vector3 &dpos, const quaternion &dorn, scalar radius,
                          coordinate_axis axis, const vector3 &p, vector3 &q) noexcept;

/**
 * Computes the closest points between a line `p(s) = p0 + s*(p1 - p0)` and a circle.
 * @param cpos Center of circle.
 * @param corn Orientation of circle.
 * @param radius Circle radius.
 * @param axis The circle lies in one of the coordinate planes in object space.
 * This axis is the normal of the plane.
 * @param p0 A point in the line.
 * @param p1 Another point in the line.
 * @param num_points Number of closest points. Can be two in case the line is
 *        nearly parallel to the plane that contains the circle and its
 *        projection onto the plane intersects the circle.
 * @param s0 Outputs the parameter where the line gives the first closest point
 *        to the circle.
 * @param rc0 First closest point in the circle.
 * @param rl0 First closest point in the line.
 * @param s1 Outputs the parameter where the line gives the second closest
 *        point to the circle.
 * @param rc1 Second closest point in the circle.
 * @param rl1 Second closest point in the line.
 * @param normal Normal vector pointing out the circle.
 * @param threshold Value used to determine whether the line is parallel to circle.
 * @return The squared distance.
 */
scalar closest_point_circle_line(
    const vector3 &cpos, const quaternion &corn, scalar radius, coordinate_axis axis,
    const vector3 &p0, const vector3 &p1, size_t &num_points,
    scalar &s0, vector3 &rc0, vector3 &rl0,
    scalar &s1, vector3 &rc1, vector3 &rl1,
    vector3 &normal, scalar threshold = support_feature_tolerance) noexcept;

/**
 * @brief Computes the closest points between two circles in 3D.
 * The circles face the x axis in object space, thus lying in the yz plane.
 * @param posA Position of center of circle A.
 * @param ornA Orientation of circle A.
 * @param radiusA Radius of circle A.
 * @param axisA Coordinate axis normal to plane of circle A in object space.
 * @param posB Position of center of circle B.
 * @param ornB Orientation of circle B.
 * @param radiusB Radius of circle B.
 * @param axisB Coordinate axis normal to plane of circle B in object space.
 * @param num_points Outputs number of closest points, 1 or 2.
 * @param rA0 Outputs the first result in A.
 * @param rB0 Outputs the first result in B.
 * @param rA1 Outputs the second result in A.
 * @param rB1 Outputs the second result in B.
 * @param normal Vector pointing from the closest point of B towards a
 * corresponding closest point on A. It is non-zero even if the points coincide,
 * providing a vector that can be used as a minimum translation vector to
 * separate the circles.
 * @return Squared distance.
 */
scalar closest_point_circle_circle(
    const vector3 &posA, const quaternion &ornA, scalar radiusA, coordinate_axis axisA,
    const vector3 &posB, const quaternion &ornB, scalar radiusB, coordinate_axis axisB,
    size_t &num_points, vector3 &rA0, vector3 &rB0, vector3 &rA1, vector3 &rB1,
    vector3 &normal);

/**
 * Constructs an orthonomal basis given one vector. In other words, given a
 * plane normal, it finds two orthogonal vectors `p` and `q` which lie in
 * the plane.
 * @param n First basis vector, plane normal.
 * @param p Outputs the first vector on the plane.
 * @param q Outputs the second vector on the plane, orthogonal to `p`.
 */
void plane_space(const vector3 &n, vector3 &p, vector3 &q);

/**
 * @brief Builds an orthonormal basis that spans a tangent space to a
 * normal vector.
 * @param n Vector normal to a surface.
 * @return Basis with `n` in the second column.
 */
matrix3x3 make_tangent_basis(const vector3 &n);

bool intersect_aabb(const vector3 &min0, const vector3 &max0,
                    const vector3 &min1, const vector3 &max1);

size_t intersect_segments(const vector2 &p0, const vector2 &p1,
                          const vector2 &q0, const vector2 &q1,
                          scalar &s0, scalar &t0,
                          scalar &s1, scalar &t1);

/**
 * @brief Intersect a line with a circle centered in the origin in 2D.
 * @param p0 One point on the line.
 * @param p1 Another point on the line.
 * @param radius Radius of circle centered in the origin.
 * @param s0 Outputs the parameter of the first intersection, the point being
 * `lerp(p0, p1, s0)`.
 * @param s1 Outputs the parameter of the second intersection, if there is one.
 * @return Number of intersections.
 */
size_t intersect_line_circle(const vector2 &p0, const vector2 &p1,
                             scalar radius, scalar &s0, scalar &s1) noexcept;

/**
 * @brief Intersect two circles in 2D.
 * @param posA Position of first circle.
 * @param radiusA Radius of first circle.
 * @param posB Position of second circle.
 * @param radiusB Radius of second circle.
 * @param res0 First intersection result.
 * @param res1 Second intersection result.
 * @return Number of intersections.
 */
size_t intersect_circle_circle(const vector2 &posA, scalar radiusA,
                               const vector2 &posB, scalar radiusB,
                               vector2 &res0, vector2 &res1) noexcept;

vector3 support_point_circle(const vector3 &pos, const quaternion &orn,
                             scalar radius, coordinate_axis axis,
                             const vector3 &dir) noexcept;

template<size_t N>
constexpr void support_point_vertices(const std::array<vector3, N> &vertices,
                              const vector3 &dir, size_t &idx, scalar &proj) noexcept {
    proj = -EDYN_SCALAR_MAX;

    for (size_t i = 0; i < N; ++i) {
        auto d = dot(vertices[i], dir);

        if (d > proj) {
            proj = d;
            idx = i;
        }
    }
}

enum class point_insertion_type {
    none,
    similar,
    append,
    replace
};

struct insertion_point_result {
    point_insertion_type type;
    size_t index;
};

insertion_point_result insertion_point_index(const vector3 *points,
                                             const scalar *depths,
                                             size_t count,
                                             size_t &num_points,
                                             const vector3 &new_point) noexcept;

template<size_t N>
insertion_point_result insertion_point_index(const std::array<vector3, N> &points,
                                             const std::array<scalar, N> &depths,
                                             size_t &num_points,
                                             const vector3 &new_point) noexcept {
    return insertion_point_index(points.data(), depths.data(), N, num_points, new_point);
}

/**
 * Finds the point closest to `p` on the surface of the axis-aligned box
 * with the given half extent if `p` is outside the box.
 * @param half_extent Half the extent on the box along each axis.
 * @param p The query point.
 * @return Point on the box surface closest to `p`, or `p` if `p` is contained
 *         in the box.
 */
vector3 closest_point_box_outside(const vector3 &half_extent, const vector3 &p) noexcept;

/**
 * Finds the point closest to the internal point `p` on the surface of the
 * axis-aligned box with the given half extent. `p` must be contained within
 * the box.
 * @param half_extent Half the extent on the box along each axis.
 * @param p The query point.
 * @param closest Outputs the closest point on the surface of the box.
 * @param normal Outputs the normal of the face where `closest` is located.
 * @return Distance between closest points.
 */
scalar closest_point_box_inside(const vector3 &half_extent, const vector3 &p,
                                vector3 &closest, vector3 &normal) noexcept;

/**
 * Intersect a line with an AABB in the Cartesian plane.
 * @param p0 Point in the line.
 * @param p1 Another point in the line.
 * @param aabb_min Minimum of AABB, i.e. lower left corner.
 * @param aabb_max Maximum of AABB, i.e. upper right corner.
 * @param s0 Outputs the parameter of one point in the line defined by `p0` and
 *        `p1` where it intersects the AABB.
 * @param s1 Outputs the parameters of another point where the line intersects
 *        the AABB.
 * @return Number of intersections in [0, 2].
 */
size_t intersect_line_aabb(const vector2 &p0, const vector2 &p1,
                           const vector2 &aabb_min, const vector2 &aabb_max,
                           scalar &s0, scalar &s1) noexcept;

/**
 * @brief Checks if a point lies inside the prism with base defined by a convex
 * polygon.
 * @param vertices An array of vertices.
 * @param indices Indices of vertices in the vertex array to be considered. They
 * should represent a convex polygon laying on a plane, oriented counter-clockwise
 * with respect to the plane normal.
 * @param normal Normal of plane that fits the convex polygon.
 * @param point Point to test.
 * @return Whether the point is inside the convex polygon or not.
 */
bool point_in_polygonal_prism(const std::vector<vector3> &vertices,
                              const std::vector<size_t> &indices,
                              const vector3 &normal, const vector3 &point) noexcept;

template<size_t N>
constexpr bool point_in_polygonal_prism(const std::array<vector3, N> &vertices,
                                        const vector3 &normal, const vector3 &point) noexcept {
    static_assert(N > 2);

    for (size_t i = 0; i < N; ++i) {
        const auto j = (i + 1) % N;
        auto &v0 = vertices[i];
        auto &v1 = vertices[j];
        auto d = v1 - v0;
        auto t = cross(d, normal);

        if (dot(point - v0, t) > EDYN_EPSILON) {
            return false;
        }
    }

    return true;
}

/**
 * @brief Test if a segment intersects an AABB.
 * @param p0 First point in the segment.
 * @param p1 Second point in the segment.
 * @param aabb_min Minimum of AABB.
 * @param aabb_max Maximum of AABB.
 * @return Whether segment intersects AABB.
 */
bool intersect_segment_aabb(vector3 p0, vector3 p1,
                            vector3 aabb_min, vector3 aabb_max) noexcept;

struct intersect_ray_cylinder_result {
    enum class kind {
        parallel_directions,
        distance_greater_than_radius,
        intersects
    };

    kind kind;
    scalar dist_sqr;
    vector3 normal;
};

/**
 * @brief Intersects a ray with a cylinder.
 * @param p0 First point in the ray.
 * @param p1 Second point in the ray.
 * @param pos Cylinder position.
 * @param orn Cylinder orientation.
 * @param radius Cylinder radius.
 * @param half_length Cylinder half length.
 * @param fraction_in Fraction where the line enters the cylinder.
 * @param fraction_out Fraction where the line exits the cylinder.
 * @return Result containing intersection situation, the distance and normal.
 */
intersect_ray_cylinder_result intersect_ray_cylinder(vector3 p0, vector3 p1,
                                                     vector3 pos, quaternion orn,
                                                     scalar radius, scalar half_length,
                                                     coordinate_axis axis,
                                                     scalar &fraction_in, scalar &fraction_out) noexcept;

/**
 * @brief Intersects a ray with a sphere.
 * @param p0 First point in the ray.
 * @param p1 Second point in the ray.
 * @param pos Sphere position.
 * @param radius Sphere radius.
 * @param t Output intersection parameter.
 * @return Whether ray intersects sphere.
 */
bool intersect_ray_sphere(vector3 p0, vector3 p1, vector3 pos, scalar radius, scalar &t) noexcept;

/**
 * @brief Intersects a ray with a triangle.
 * @param p0 First point in the ray.
 * @param p1 Second point in the ray.
 * @param vertices Vertex positions.
 * @param normal Triangle normal.
 * @param t Output intersection parameter.
 * @return Whether ray intersects triangle.
 */
bool intersect_segment_triangle(const vector3 &p0, const vector3 &p1,
                                const std::array<vector3, 3> &vertices,
                                const vector3 &normal, scalar &t) noexcept;

/**
 * @brief Tests whether two edges would generate a face in the Minkowski
 * difference of two convex shapes. This utility function is part of the
 * Gauss Map trick presented by Dirk Gregorius at GDC 2013:
 * "Physics for Game Programmers: The Separating Axis Test between Convex Polyhedra"
 * https://www.gdcvault.com/play/1017646/Physics-for-Game-Programmers-The
 * The key idea is: only pairs of edges that generate a face in the Minkowski
 * difference could ever produce a relevant separating axis.
 * @remark C and D are assumed as negated because generally they would have to
 * be negated by the caller. This is done for performance reasons (this is a
 * profiled optimization).
 * @param A Normal of first incident face on the first edge.
 * @param B Normal of second incident face on the first edge.
 * @param C_neg Normal of first incident face on the second edge, negated.
 * @param D_neg Normal of second incident face on the second edge, negated.
 * @param B_x_A Cross product of B and A, which is parallel to the first edge.
 * @param D_x_C Cross product of D and C, which is parallel to the second edge.
 */
bool edges_generate_minkowski_face(vector3 A, vector3 B, vector3 C_neg, vector3 D_neg, vector3 B_x_A, vector3 D_x_C);

}

#endif // EDYN_MATH_GEOM_HPP
