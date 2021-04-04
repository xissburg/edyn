#ifndef EDYN_SHAPES_TRIANGLE_SHAPE_HPP
#define EDYN_SHAPES_TRIANGLE_SHAPE_HPP

#include "edyn/math/vector3.hpp"
#include "edyn/comp/aabb.hpp"
#include <array>
#include <cstdint>

namespace edyn {

/**
 * The three types of triangle features. Their integer value is the number of
 * vertices in that feature minus one.
 */
enum class triangle_feature {
    vertex = 0,
    edge,
    face
};

using triangle_vertices = std::array<vector3, 3>;
using triangle_edges = std::array<vector3, 3>;

/**
 * Holds information of one triangle in a mesh. Includes relevant adjacency
 * information.
 */
struct triangle_shape {
    // Position of vertices.
    triangle_vertices vertices;

    // Whether an edge is concave in the mesh.
    std::array<bool, 3> is_concave_edge;

    // Whether a vertex is concave in the mesh, which is true when any of the
    // two edges that share a vertex is concave.
    std::array<bool, 3> is_concave_vertex;

    // Cosine of the angle of each edge, i.e. the angle between the normal of
    // this triangle and the normal of the other triangle that is a neighbor
    // through the edge.
    std::array<scalar, 3> cos_angles;
    
    // Edge vectors, i.e. `vertices[(i+1)%3] - vertices[i]` for the ith edge.
    std::array<vector3, 3> edges;

    // Vectors orthogonal to the edges and triangle normal, pointing outside
    // the triangle.
    std::array<vector3, 3> edge_tangents;

    // The triangle normal.
    vector3 normal;

    void update_computed_properties();

    /**
     * @brief Returns whether collision with an edge at a certain direction
     * should not be considered.
     * @param idx Edge index.
     * @param dir Collision normal.
     * @return Whether the collision should be ignored.
     */
    bool ignore_edge(size_t idx, const vector3 &dir) const;

    /**
     * @brief Returns whether collision with a vertex in a certain direction
     * should not be considered.
     * @param idx Vertex index.
     * @param dir Collision normal.
     * @return Whether the collision should be ignored.
     */
    bool ignore_vertex(size_t idx, const vector3 &dir) const;

    /**
     * @brief Returns whether collision with a specific triangle feature
     * in a certain direction should not be considered.
     * @param tri_feature The triangle feature kind.
     * @param idx Feature index.
     * @param dir Collision normal.
     * @return Whether the collision should be ignored.
     */
    bool ignore_feature(triangle_feature tri_feature, 
                        size_t idx, const vector3 &dir) const;
};

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

size_t get_triangle_feature_num_vertices(triangle_feature feature);

size_t get_triangle_feature_num_edges(triangle_feature feature);

AABB get_triangle_aabb(const triangle_vertices &vertices);

}

#endif // EDYN_SHAPES_TRIANGLE_SHAPE_HPP