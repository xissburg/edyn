#ifndef EDYN_SHAPES_TRIANGLE_MESH_HPP
#define EDYN_SHAPES_TRIANGLE_MESH_HPP

#include <cstdint>
#include <vector>
#include "edyn/math/vector3.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/comp/aabb.hpp"
#include "edyn/shapes/triangle_shape.hpp"
#include "edyn/collision/static_tree.hpp"

namespace edyn {

/**
 * @brief A triangle mesh. Includes adjacency information and a tree to
 * accelerate closest point queries.
 */
class triangle_mesh {
    void initialize();
    void calculate_face_normals();
    void init_edge_indices();
    void calculate_edge_normals();
    void init_vertex_edge_indices();
    void init_face_edge_indices();
    void calculate_concave_vertices();
    void calculate_concave_edges();
    void build_tree();

public:
    using index_type = uint16_t;

    size_t num_triangles() const {
        return m_indices.size();
    }

    AABB get_aabb() const {
        return m_tree.root_aabb();
    }

    triangle_vertices get_triangle_vertices(size_t tri_idx) const;

    template<typename Func>
    void visit(const AABB &aabb, Func func) const {
        constexpr auto inset = vector3 {
            -contact_breaking_threshold,
            -contact_breaking_threshold,
            -contact_breaking_threshold
        };

        m_tree.visit(aabb.inset(inset), func);
    }

    template<typename Func>
    void visit_all(Func func) const {
        for (size_t i = 0; i < num_triangles(); ++i) {
            auto verts = triangle_vertices{
                m_vertices[m_indices[i][0]],
                m_vertices[m_indices[i][1]],
                m_vertices[m_indices[i][2]]
            };

            func(i, verts);
        }
    }

    bool ignore_vertex(size_t vertex_idx, const vector3 &dir) const;

    bool ignore_edge(size_t edge_idx, const vector3 &dir) const;

private:
    // Vertex positions.
    std::vector<vector3> m_vertices;

    // Vertex indices for each triangular face. Each element represents the
    // vertex indices of one triangle.
    std::vector<std::array<index_type, 3>> m_indices;

    // Face normals.
    std::vector<vector3> m_normals;

    // Vertex indices for each unique edge. Each pair of values represent the
    // vertex indices for one edge.
    std::vector<std::array<index_type, 2>> m_edge_indices;

    // Vectors that are orthogonal to an edge and parallel to the faces that
    // share this edge. They're stored in pairs where each value is a vector
    // that points towards each face. For perimetral edges, both values are the
    // same and point towards the single face that contains this edge.
    std::vector<std::array<vector3, 2>> m_edge_normals;

    // Indices of edges that share a vertex. There is a sequence of N
    // indices for each vertex. The range of the sequence is stored in
    // `vertex_edge_index_ranges`.
    std::vector<index_type> m_vertex_edge_indices;

    // Each pair of values represent the start of the edge index list in the
    // `vertex_edge_indices` array followed by the number of indices, i.e.
    // the number of edges sharing a vertex.
    std::vector<std::array<index_type, 2>> m_vertex_edge_index_ranges;

    // Each element represents the indices of the three edges of a face.
    std::vector<std::array<index_type, 3>> m_face_edge_indices;

    // Indices of the two faces that share the i-th edge. Perimetral edges will
    // have the same value for both faces.
    std::vector<std::array<index_type, 2>> m_edge_face_indices;

    // Whether an edge is concave.
    std::vector<bool> m_is_concave_edge;

    // Whether a vertex is concave.
    std::vector<bool> m_is_concave_vertex;

    static_tree m_tree;
};

}

#endif // EDYN_SHAPES_TRIANGLE_MESH_HPP