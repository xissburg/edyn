#ifndef EDYN_UTIL_SHAPE_UTIL_HPP
#define EDYN_UTIL_SHAPE_UTIL_HPP

#include "edyn/math/scalar.hpp"
#include "edyn/math/vector2.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/comp/aabb.hpp"
#include <vector>
#include <cstdint>
#include <string>

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
void make_plane_mesh(scalar extent_x, scalar extent_z, 
                     size_t num_vertices_x, size_t num_vertices_z, 
                     std::vector<vector3> &vertices, std::vector<uint16_t> &indices);

/**
 * @brief Loads a mesh from a *.obj file.
 * @param path Path to file.
 * @param vertices Array to be filled with vertices.
 * @param indices Array to be filled with indices for each face.
 * @param faces Array to be filled with an index of where the vertices
 * of a face starts in the `indices` array and the vertex count of the 
 * face, i.e. a sequence of pairs of (index of first vertex index, 
 * number of vertices).
 * @return Success or failure.
 */
bool load_mesh_from_obj(const std::string &path, 
                        std::vector<vector3> &vertices, 
                        std::vector<uint16_t> &indices,
                        std::vector<uint16_t> &faces);

/**
 * @brief Loads a triangle mesh from a *.obj file which must've been
 * triangulated during export.
 * @param path Path to file.
 * @param vertices Array to be filled with vertices.
 * @param indices Array to be filled with indices for each triangle.
 * @return Success or failure.
 */
bool load_tri_mesh_from_obj(const std::string &path, 
                            std::vector<vector3> &vertices, 
                            std::vector<uint16_t> &indices);

/**
 * @brief Calculates a point on a axis-aligned box that's furthest along
 * a given direction, i.e. support point.
 * @param half_extents Extents of the box, halved.
 * @param dir A direction vector (non-zero).
 * @return A support point.
 */
vector3 support_point_box(const vector3 &half_extents, const vector3 &dir);

/**
 * @brief Calculates the AABB of a transformed AABB.
 * @param aabb The AABB.
 * @param pos Position of AABB.
 * @param orn Orientation of AABB.
 * @return AABB of the given AABB with transformation applied.
 */
AABB aabb_of_aabb(const AABB &aabb, const vector3 &pos, const quaternion &orn);

/**
 * @brief Calculates the AABB of a set of points.
 * @param points A point cloud.
 * @return AABB of point set.
 */
AABB point_cloud_aabb(const std::vector<vector3> &points);

/**
 * @brief Calculates the AABB of a set of points with a transformation.
 * @param points A point cloud.
 * @param pos Position offset applied to all points.
 * @param orn Orientation of point cloud.
 * @return AABB of point set.
 */
AABB point_cloud_aabb(const std::vector<vector3> &points, 
                      const vector3 &pos, const quaternion &orn);

/**
 * @brief Returns a point in the set that's furthest away in the given
 * direction, i.e. a support point.
 * @param points A point cloud.
 * @param dir A direction vector (non-zero).
 * @return Support point.
 */
vector3 point_cloud_support_point(const std::vector<vector3> &points, const vector3 &dir);

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
 * @brief Calculates a convex hull of a set of points.
 * @param points A point cloud.
 * @param tolerance Controls how points are ignored based on colinearity.
 * @return An array of indices of the convex hull vertices oriented
 * counter-clockwise. It can be modified as a result of this call.
 */
std::vector<size_t> calculate_convex_hull(std::vector<vector2> &points, scalar tolerance);

/**
 * @brief Checks if a point lies inside a convex polygon.
 * @param vertices Vertices of a convex polygon oriented counter-clockwise.
 * @param point The point to test.
 * @return Whether the point is inside the convex polygon or not.
 */
bool point_inside_convex_polygon(const std::vector<vector2> &vertices, const vector2 &point);

/**
 * @brief Swaps vertices to make the winding of the sequence (v0, v1, v2) 
 * counter-clockwise.
 * @param v0 A vertex of a triangle.
 * @param v1 Another vertex of a triangle.
 * @param v2 Yet another vertex of a triangle.
 */
void sort_triangle_ccw(vector2 &v0, vector2 &v1, vector2 &v2);

}

#endif // EDYN_UTIL_SHAPE_UTIL_HPP
