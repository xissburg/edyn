#ifndef EDYN_UTIL_SHAPE_UTIL_HPP
#define EDYN_UTIL_SHAPE_UTIL_HPP

#include "edyn/math/vector2.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/math/vector2_3_util.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/triangle.hpp"
#include "edyn/math/coordinate_axis.hpp"
#include <vector>
#include <cstdint>

namespace edyn {

/**
 * @brief Builds vertices and indices for a planar triangle mesh.
 * @param extent_x Extent of plane along the x-axis.
 * @param extent_z Extent of plane along the z-axis.
 * @param num_vertices_x Number of vertices along x-axis.
 * @param num_vertices_z Number of vertices along z-axis.
 * @param vertices Array to be filled with vertices.
 * @param indices Array to be filled with indices for each triangle.
 */
template<typename IndexType>
void make_plane_mesh(scalar extent_x, scalar extent_z,
                     size_t num_vertices_x, size_t num_vertices_z,
                     std::vector<vector3> &vertices, std::vector<IndexType> &indices) {
    const auto half_extent_x = extent_x * scalar(0.5);
    const auto half_extent_z = extent_z * scalar(0.5);
    const auto quad_size_x = extent_x / (num_vertices_x - 1);
    const auto quad_size_z = extent_z / (num_vertices_z - 1);

    for (size_t j = 0; j < num_vertices_z; ++j) {
        auto z = -half_extent_z + j * quad_size_z;
        for (size_t i = 0; i < num_vertices_x; ++i) {
            vertices.push_back({-half_extent_x + i * quad_size_x, 0, z});
        }
    }

    for (size_t j = 0; j < num_vertices_z - 1; ++j) {
        for (size_t i = 0; i < num_vertices_x - 1; ++i) {
            indices.push_back(j * num_vertices_x + i + 1);
            indices.push_back(j * num_vertices_x + i);
            indices.push_back((j + 1) * num_vertices_x + i);

            indices.push_back((j + 1) * num_vertices_x + i);
            indices.push_back((j + 1) * num_vertices_x + i + 1);
            indices.push_back(j * num_vertices_x + i + 1);
        }
    }
}

/**
 * @brief Builds a convex mesh in the shape of a box.
 * @param half_extents Half of the extent of the box in each axis.
 * @param vertices Fills it with vertices.
 * @param indices Fills it with sequences of vertex indices for each face.
 * @param faces Fills it with [first index, count] pairs.
 */
void make_box_mesh(const vector3 &half_extents,
                   std::vector<vector3> &vertices,
                   std::vector<uint32_t> &indices,
                   std::vector<uint32_t> &faces);

/**
 * @brief Calculates a point on a axis-aligned box that's furthest along
 * a given direction, i.e. support point.
 * @param half_extents Extents of the box, halved.
 * @param dir A direction vector (non-zero).
 * @return A support point.
 */
vector3 support_point_box(const vector3 &half_extents, const vector3 &dir);

/**
 * @brief Returns a point in the set that's furthest away in the given
 * direction, i.e. a support point.
 * @param points A point cloud.
 * @param dir A direction vector (non-zero).
 * @return Support point.
 */
vector3 point_cloud_support_point(const std::vector<vector3> &points, const vector3 &dir);

/**
 * @brief Returns a point in the set that's furthest away in the given
 * direction, i.e. a support point.
 * @param first Iterator to the first element of the point cloud.
 * @param last Iterator to the last element of the point cloud.
 * @param dir A direction vector (non-zero).
 * @return Support point.
 */
template<typename It>
vector3 point_cloud_support_point(It first, It last, const vector3 &dir) {
    auto sup = vector3_zero;
    auto max_proj = -EDYN_SCALAR_MAX;

    for (auto it = first; it != last; ++it) {
        const auto &point = *it;
        auto proj = dot(point, dir);

        if (proj > max_proj) {
            max_proj = proj;
            sup = point;
        }
    }

    return sup;
}


/**
 * @brief Calculates the maximum projection of all vertices along the given
 * direction. Uses adjacency information to achieve `O(log n)` complexity.
 * @param vertices Vertices of a convex polyhedron.
 * @param neighbors_start List of indices where the list of neighbors start for
 * each vertex in the `neighbor_indices` vector. See `convex_mesh:neighbors_start`
 * for further details.
 * @param neighbor_indices List of neighboring vertex indices. See
 * `convex_mesh:neighbors_start` for further details.
 * @param dir A direction vector (non-zero).
 * @return The maximal projection.
 */
scalar polyhedron_support_projection(const std::vector<vector3> &vertices,
                                     const std::vector<uint32_t> &neighbors_start,
                                     const std::vector<uint32_t> &neighbor_indices,
                                     const vector3 &dir);

/**
 * @brief Calculates the maximum projection of all points along the given
 * direction.
 * @tparam It Type of iterator of a `vector3` container.
 * @param first Iterator to the first element of the point cloud.
 * @param last Iterator to the last element of the point cloud.
 * @param dir A direction vector (non-zero).
 * @return The maximal projection.
 */
template<typename It>
scalar point_cloud_support_projection(It first, It last, const vector3 &dir) {
    auto max_proj = -EDYN_SCALAR_MAX;

    for (auto it = first; it != last; ++it) {
        const auto &point = *it;
        auto proj = dot(point, dir);
        max_proj = std::max(proj, max_proj);
    }

    return max_proj;
}

/**
 * @brief Calculates the maximum projection of a vector of points along the
 * given direction.
 * @param points A point cloud.
 * @param dir A direction vector (non-zero).
 * @return The maximal projection.
 */
scalar point_cloud_support_projection(const std::vector<vector3> &points, const vector3 &dir);

/**
 * @brief Calculates a convex hull of a set of points.
 * @param points A point cloud.
 * @param tolerance Controls how points are ignored based on collinearity.
 * @return An array of indices of the convex hull vertices oriented
 * counter-clockwise. It can be modified as a result of this call.
 */
std::vector<size_t> calculate_convex_hull(const std::vector<vector2> &points, scalar tolerance);

/**
 * @brief Checks if a point lies inside a convex polygon.
 * @param vertices Vertices of a convex polygon oriented counter-clockwise.
 * @param point The point to test.
 * @return Whether the point is inside the convex polygon or not.
 */
bool point_inside_convex_polygon(const std::vector<vector2> &vertices, const vector2 &point);

/**
 * @brief Checks whether the given triangle vertices are sorted in a
 * counter-clockwise winding order.
 * @param v0 A vertex of a triangle.
 * @param v1 Another vertex of a triangle.
 * @param v2 Yet another vertex of a triangle.
 * @return Whether vertices are sorted CCW.
 */
bool is_triangle_ccw(const vector2 &v0, const vector2 &v1, const vector2 &v2);

/**
 * @brief Swaps vertices to make the winding of the sequence (v0, v1, v2)
 * counter-clockwise.
 * @param v0 A vertex of a triangle.
 * @param v1 Another vertex of a triangle.
 * @param v2 Yet another vertex of a triangle.
 * @return Whether vertices have been sorted.
 */
bool sort_triangle_ccw(vector2 &v0, vector2 &v1, vector2 &v2);

/**
 * Useful information describing a support polygon on a polyhedron.
 */
struct support_polygon {
    // Vertices in world space.
    std::vector<vector3> vertices;
    // Vertices on the 2D contact plane, i.e. `vertices` transformed into
    // the contact space with `to_vector2_xz` applied to them.
    std::vector<vector2> plane_vertices;
    // Indices of the vertices that are in the convex hull of the polygon,
    // oriented counter-clockwise.
    std::vector<size_t> hull;
    // Contact plane origin.
    vector3 origin;
    // Contact plane basis, where the second column is the plane normal and
    // the others span the contact plane.
    matrix3x3 basis;
};

/**
 * @brief Finds the polygon in a point cloud that's furthest away in the given
 * direction. The result could be an edge, i.e. only two vertices.
 * @tparam It Type of iterator of a `vector3` container.
 * @param first Iterator to the first element of the point cloud.
 * @param last Iterator to the last element of the point cloud.
 * @param offset Vector to be added to each point during calculations.
 * @param dir A direction vector (non-zero).
 * @param projection The support projection along the given direction, i.e. the
 * value returned by `point_cloud_support_projection(first, last, +/-dir)`.
 * @param positive_side Whether the direction points towards or away of the
 * point cloud. Must be true if it points towards.
 * @param tolerance The distance from the projection boundary which decides
 * whether the vertex is part of the support polygon.
 */
template<typename It>
support_polygon point_cloud_support_polygon(It first, It last,
                                            const vector3 &offset,
                                            const vector3 &dir,
                                            scalar projection,
                                            const bool positive_side,
                                            scalar tolerance) {
    auto polygon = support_polygon{};
    polygon.origin = dir * projection;
    // Basis tangent to the contact plane so calculations can be done in tangent space.
    polygon.basis = make_tangent_basis(dir);

    const auto zero_offset = offset == vector3_zero;

    for (auto it = first; it != last; ++it) {
        vector3 vertex_world;
        if (zero_offset) {
            vertex_world = *it;
        } else {
            vertex_world = *it + offset;
        }

        auto is_in_boundary = positive_side ?
            dot(vertex_world, dir) < projection + tolerance :
            dot(vertex_world, dir) > projection - tolerance;

        if (!is_in_boundary) continue;

        polygon.vertices.push_back(vertex_world);
        // Find vertex in the contact plane and convert to 2D space.
        auto vertex_tangent = to_object_space(vertex_world, polygon.origin, polygon.basis);
        auto vertex_plane = to_vector2_xz(vertex_tangent);
        polygon.plane_vertices.push_back(vertex_plane);
    }

    EDYN_ASSERT(!polygon.vertices.empty() && !polygon.plane_vertices.empty());

    const auto hull_tolerance = scalar(0.001);
    polygon.hull = calculate_convex_hull(polygon.plane_vertices, hull_tolerance);

    return polygon;
}

/**
 * @brief Finds a point on the boundary of a convex polygon that's closest to a
 * point `p` located outside the polygon.
 * @param vertices Array of vertices.
 * @param indices Sequence of indices of vertices that form a convex polygon in
 * the `vertices` array.
 * @param p Query point.
 * @param closest Point on the boundary of the polygon that's closest to `p`.
 * @return False if `p` is inside the polygon.
 */
bool closest_point_convex_polygon(const std::vector<vector2> &vertices,
                                  const std::vector<size_t> &indices,
                                  const vector2 &p, vector2 &closest);

/**
 * @brief Finds a point on the boundary of a support polygon that's closest to a
 * point `p` located outside the polygon.
 * @param polygon A support polygon.
 * @param p Query point.
 * @param closest Point on the boundary of the polygon that's closest to `p`.
 * @return False if `p` is inside the polygon.
 */
bool closest_point_polygon(const support_polygon &polygon,
                           const vector2 &p, vector2 &closest);

/**
 * @brief Calculates the maximum projection of a capsule shape along the
 * given direction.
 * @param v0 Location of the first vertex of the capsule in world-space.
 * @param v1 Location of the second vertex of the capsule in world-space.
 * @param radius Radius of capsule.
 * @param dir A direction vector (non-zero).
 * @return The maximal projection.
 */
scalar capsule_support_projection(const vector3 &v0, const vector3 &v1,
                                  scalar radius, const vector3 &dir);

/**
 * @brief Calculates the maximum projection of a capsule shape along the
 * given direction.
 * @param vertices The two vertices of the capsule in world-space.
 * @param radius Radius of capsule.
 * @param dir A direction vector (non-zero).
 * @return The maximal projection.
 */
scalar capsule_support_projection(const std::array<vector3, 2> &vertices,
                                  scalar radius, const vector3 &dir);

/**
 * @brief Calculates a point that's furthest along the given direction on a
 * cylinder centered in the origin oriented along the x axis.
 * @param radius Cylinder radius.
 * @param half_length Half the length of the cylinder.
 * @param axis The axis of the cylinder.
 * @param dir A direction vector (non-zero).
 * @return A support point.
 */
vector3 cylinder_support_point(scalar radius, scalar half_length, coordinate_axis axis, const vector3 &dir);

/**
 * @brief Calculates a point that's furthest along the given direction on an
 * oriented cylinder centered in the origin. The cylinder's main axis is x.
 * @param radius Cylinder radius.
 * @param half_length Half the length of the cylinder.
 * @param axis The axis of the cylinder.
 * @param orn The cylinder's orientation.
 * @param dir A direction vector (non-zero).
 * @return A support point.
 */
vector3 cylinder_support_point(scalar radius, scalar half_length, coordinate_axis axis,
                               const quaternion &orn, const vector3 &dir);

/**
 * @brief Calculates a point that's furthest along the given direction on an
 * cylinder where the cylinder's main axis is x.
 * @param radius Cylinder radius.
 * @param half_length Half the length of the cylinder.
 * @param axis The axis of the cylinder.
 * @param pos The cylinder's position.
 * @param orn The cylinder's orientation.
 * @param dir A direction vector (non-zero).
 * @return A support point.
 */
vector3 cylinder_support_point(scalar radius, scalar half_length, coordinate_axis axis,
                               const vector3 &pos, const quaternion &orn, const vector3 &dir);

/**
 * @brief Calculates the maximum projection of a cylinder along the given
 * direction where the cylinder's main axis is x.
 * @param radius Cylinder radius.
 * @param half_length Half the length of the cylinder.
 * @param axis The axis of the cylinder.
 * @param pos The cylinder's position.
 * @param orn The cylinder's orientation.
 * @param dir A direction vector (non-zero).
 * @return The maximal projection.
 */
scalar cylinder_support_projection(scalar radius, scalar half_length, coordinate_axis axis,
                                   const vector3 &pos, const quaternion &orn, const vector3 &dir);

/**
 * @brief Calculates the centroid of a mesh.
 * @param vertices Vertex positions.
 * @param indices Indices of vertices of each face in the `vertices` array.
 * @param faces Sequence of pairs containing the index of the first index of a
 * face in the `indices` array followed by the number of vertices of that face.
 * @return The centroid.
 */
vector3 mesh_centroid(const std::vector<vector3> &vertices,
                      const std::vector<uint32_t> &indices,
                      const std::vector<uint32_t> &faces);

struct collision_feature;

/**
 * @brief Get a triangle mesh feature index from the local index of a triangle
 * feature.
 * @param mesh The mesh indices should be obtained from.
 * @param tri_idx Triangle index in the mesh.
 * @param tri_feature Triangle feature.
 * @param tri_feature_index Index of triangle feature.
 * @return Index of feature in the triangle mesh.
 */
size_t get_triangle_mesh_feature_index(const triangle_mesh &mesh, size_t tri_idx,
                                       triangle_feature tri_feature, size_t tri_feature_idx);

}

#endif // EDYN_UTIL_SHAPE_UTIL_HPP
