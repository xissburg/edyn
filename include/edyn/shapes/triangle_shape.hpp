#ifndef EDYN_SHAPES_TRIANGLE_SHAPE_HPP
#define EDYN_SHAPES_TRIANGLE_SHAPE_HPP

#include "edyn/math/vector3.hpp"
#include <array>
#include <cstdint>

namespace edyn {

enum triangle_feature {
    TRIANGLE_FEATURE_VERTEX,
    TRIANGLE_FEATURE_EDGE,
    TRIANGLE_FEATURE_FACE
};

using triangle_vertices = std::array<vector3, 3>;
using triangle_edges = std::array<vector3, 3>;

/**
 * Checks whether point `p` is contained within the infinite prism with 
 * triangular base defined by the given `vertices` and direction `normal`.
 */
bool point_in_triangle(const triangle_vertices &, 
                       const vector3 &normal, 
                       const vector3 &p);

triangle_edges get_triangle_edges(const triangle_vertices &vertices);

/**
 * Gets the greatest projection of the triangle onto the given axis
 * along with the feature present at the extreme.
 */
void get_triangle_support_feature(const triangle_vertices &vertices, 
                                  const vector3 &axis_pos, const vector3 &axis_dir,
                                  triangle_feature &tri_feature,
                                  uint8_t &tri_feature_index,
                                  scalar &projection);

}

#endif // EDYN_SHAPES_TRIANGLE_SHAPE_HPP