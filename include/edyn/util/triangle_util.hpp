#ifndef EDYN_SHAPES_TRIANGLE_UTIL_HPP
#define EDYN_SHAPES_TRIANGLE_UTIL_HPP

#include "edyn/math/vector3.hpp"
#include "edyn/comp/aabb.hpp"
#include <array>
#include <cstdint>

namespace edyn {

/**
 * The three types of triangle features. Their integer value is the number of
 * vertices in that feature minus one.
 */
enum class triangle_feature : uint8_t {
    vertex = 0,
    edge,
    face
};

using triangle_vertices = std::array<vector3, 3>;
using triangle_edges = std::array<vector3, 3>;
class triangle_mesh;

/**
 * Checks whether point `p` is contained within the infinite prism with
 * triangular base defined by the given `vertices` and direction `normal`.
 */
bool point_in_triangle(const triangle_vertices &,
                       const vector3 &normal,
                       const vector3 &p);

triangle_edges get_triangle_edges(const triangle_vertices &);

/**
 * Gets the greatest projection of the triangle onto the given axis
 * along with the feature present at the extreme.
 */
void get_triangle_support_feature(const triangle_vertices &,
                                  const vector3 &axis_pos, const vector3 &axis_dir,
                                  triangle_feature &tri_feature,
                                  size_t &tri_feature_index,
                                  scalar &projection, scalar threshold);

vector3 get_triangle_support_point(const triangle_vertices &, const vector3 &dir);

scalar get_triangle_support_projection(const triangle_vertices &, const vector3 &dir);

size_t get_triangle_feature_num_vertices(triangle_feature feature);

size_t get_triangle_feature_num_edges(triangle_feature feature);

AABB get_triangle_aabb(const triangle_vertices &vertices);

// This is used in triangle mesh collision detection to project a separating
// axis into the voronoi region of the support feature of a triangle, thus
// preventing contacts from pointing towards an invalid direction.
vector3 clip_triangle_separating_axis(vector3 sep_axis, const triangle_mesh &mesh,
                                      size_t tri_idx, const std::array<vector3, 3> &tri_vertices,
                                      const vector3 &tri_normal, triangle_feature tri_feature,
                                      size_t tri_feature_index);

}

#endif // EDYN_SHAPES_TRIANGLE_UTIL_HPP
