#ifndef EDYN_SHAPES_TRIANGLE_MESH_HPP
#define EDYN_SHAPES_TRIANGLE_MESH_HPP

#include <cstdint>
#include <vector>
#include "edyn/math/vector3.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/comp/aabb.hpp"
#include "edyn/util/triangle_util.hpp"
#include "edyn/collision/static_tree.hpp"
#include "edyn/util/commutative_pair.hpp"
#include "edyn/util/flat_nested_array.hpp"

namespace edyn {

namespace detail {
    struct submesh_builder;
}

/**
 * @brief A triangle mesh. Includes adjacency information and a tree to
 * accelerate closest point queries.
 */
class triangle_mesh {
public:
    void initialize();
    void calculate_face_normals();
    void init_edge_indices();
    void calculate_edge_normals();
    void init_vertex_tangents();
    void calculate_convex_edges();
    void build_triangle_tree();

public:
    using index_type = uint32_t;

    template<typename It>
    void insert_vertices(It first, It last) {
        m_vertices.reserve(std::distance(first, last));
        m_vertices.insert(m_vertices.end(), first, last);
    }

    template<typename It>
    void insert_indices(It first, It last) {
        auto num_triangles = std::distance(first, last) / 3;
        m_indices.reserve(num_triangles);

        for (auto it = first; it != last; it += 3) {
            m_indices.push_back({*it, *(it + 1), *(it + 2)});
        }
    }

    size_t num_vertices() const {
        return m_vertices.size();
    }

    size_t num_edges() const {
        return m_edge_vertex_indices.size();
    }

    size_t num_triangles() const {
        return m_indices.size();
    }

    AABB get_aabb() const {
        return m_triangle_tree.root_aabb();
    }

    vector3 get_vertex_position(size_t vertex_idx) const {
        EDYN_ASSERT(vertex_idx < m_vertices.size());
        return m_vertices[vertex_idx];
    }

    triangle_vertices get_triangle_vertices(size_t tri_idx) const;

    vector3 get_triangle_normal(size_t tri_idx) const {
        EDYN_ASSERT(tri_idx < m_normals.size());
        return m_normals[tri_idx];
    }

    std::array<vector3, 2> get_edge_vertices(size_t edge_idx) const {
        EDYN_ASSERT(edge_idx < m_edge_vertex_indices.size());
        return {
            m_vertices[m_edge_vertex_indices[edge_idx][0]],
            m_vertices[m_edge_vertex_indices[edge_idx][1]]
        };
    }

    auto get_edge_face_indices(size_t edge_idx) const {
        EDYN_ASSERT(edge_idx < m_edge_face_indices.size());
        return m_edge_face_indices[edge_idx];
    }

    auto get_edge_normals(size_t edge_idx) const {
        EDYN_ASSERT(edge_idx < m_edge_normals.size());
        return m_edge_normals[edge_idx];
    }

    /**
     * @brief Returns the normals of the faces that share an edge
     * which is assumed to be convex.
     * @param edge_idx Convex edge index.
     * @return Two adjacent face normals.
     */
    std::array<vector3, 2> get_convex_edge_face_normals(size_t edge_idx) const ;

    template<typename Func>
    void visit_triangles(const AABB &aabb, Func func) const {
        m_triangle_tree.query(aabb, [&] (auto tree_node_idx) {
            auto tri_idx = m_triangle_tree.get_node(tree_node_idx).id;
            func(tri_idx);
        });
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

    bool in_vertex_voronoi(size_t vertex_idx, const vector3 &dir) const;

    bool in_edge_voronoi(size_t edge_idx, const vector3 &dir) const;

    bool is_convex_edge(size_t edge_idx) const {
        EDYN_ASSERT(edge_idx < m_is_convex_edge.size());
        return m_is_convex_edge[edge_idx];
    }

    bool is_boundary_edge(size_t edge_idx) const {
        EDYN_ASSERT(edge_idx < m_is_boundary_edge.size());
        return m_is_boundary_edge[edge_idx];
    }

    bool ignore_triangle_feature(size_t tri_idx, triangle_feature tri_feature,
                                 size_t feature_idx, const vector3 &dir) const;

    index_type get_face_vertex_index(size_t tri_idx, size_t vertex_idx) const {
        EDYN_ASSERT(tri_idx < m_face_edge_indices.size());
        EDYN_ASSERT(vertex_idx < 3);
        return m_indices[tri_idx][vertex_idx];
    }

    index_type get_face_edge_index(size_t tri_idx, size_t edge_idx) const {
        EDYN_ASSERT(tri_idx < m_face_edge_indices.size());
        EDYN_ASSERT(edge_idx < 3);
        return m_face_edge_indices[tri_idx][edge_idx];
    }

    template<typename Archive>
    friend void serialize(Archive &, triangle_mesh &);
    friend size_t serialization_sizeof(const triangle_mesh &);
    friend struct detail::submesh_builder;

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
    std::vector<commutative_pair<index_type>> m_edge_vertex_indices;

    // Indices of edges for each vertex. Each element is a list of indices of
    // edges that share the vertex.
    flat_nested_array<index_type> m_vertex_edge_indices;

    // Vectors that are orthogonal to an edge and parallel to the faces that
    // share this edge. They're stored in pairs where each value is a vector
    // that points towards each face. For perimetral edges, both values are the
    // same and point towards the single face that contains this edge.
    std::vector<std::array<vector3, 2>> m_edge_normals;

    // Vectors that are tangent to a vertex. More specifically, these are the
    // directions of the edges that depart from a vertex.
    flat_nested_array<vector3> m_vertex_tangents;

    // Each element represents the indices of the three edges of a face.
    std::vector<std::array<index_type, 3>> m_face_edge_indices;

    // Indices of the two faces that share the i-th edge. Perimetral edges will
    // have the same value for both faces.
    std::vector<std::array<index_type, 2>> m_edge_face_indices;

    // Indicates whether an edge is at the boundary. These edges are associated
    // with a single triangle.
    std::vector<bool> m_is_boundary_edge;

    // Whether an edge is convex.
    std::vector<bool> m_is_convex_edge;

    static_tree m_triangle_tree;
};

}

#endif // EDYN_SHAPES_TRIANGLE_MESH_HPP
