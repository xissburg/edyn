#ifndef EDYN_SHAPES_TRIANGLE_MESH_HPP
#define EDYN_SHAPES_TRIANGLE_MESH_HPP

#include <vector>
#include <cstdint>
#include "edyn/config/config.h"
#include "edyn/math/math.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/triangle.hpp"
#include "edyn/comp/aabb.hpp"
#include "edyn/comp/material.hpp"
#include "edyn/collision/static_tree.hpp"
#include "edyn/core/unordered_pair.hpp"
#include "edyn/core/flat_nested_array.hpp"

namespace edyn {

namespace detail {
    struct submesh_builder;
}

/**
 * @brief A triangle mesh. Includes adjacency information and a tree to
 * accelerate closest point queries.
 */
class triangle_mesh {
    void calculate_face_normals();
    void init_edge_indices();
    void calculate_adjacent_normals();
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

    template<typename It>
    void insert_friction_coefficients(It first, It last) {
        m_friction.clear();
        m_friction.insert(m_friction.end(), first, last);
    }

    template<typename It>
    void insert_restitution_coefficients(It first, It last) {
        m_restitution.clear();
        m_restitution.insert(m_restitution.end(), first, last);
    }

    void initialize();

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

    std::array<index_type, 2> get_edge_vertex_indices(size_t edge_idx) const {
        EDYN_ASSERT(edge_idx < m_edge_vertex_indices.size());
        return {
            m_edge_vertex_indices[edge_idx][0],
            m_edge_vertex_indices[edge_idx][1]
        };
    }

    auto get_edge_face_indices(size_t edge_idx) const {
        EDYN_ASSERT(edge_idx < m_edge_face_indices.size());
        return m_edge_face_indices[edge_idx];
    }

    template<typename Func>
    void visit_triangles(const AABB &aabb, Func func) const {
        m_triangle_tree.query(aabb, [&](auto tree_node_idx) {
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

    template<typename Func>
    void raycast(const vector3 &p0, const vector3 &p1, Func func) const {
        m_triangle_tree.raycast(p0, p1, [&](auto tree_node_idx) {
            auto tri_idx = m_triangle_tree.get_node(tree_node_idx).id;
            func(tri_idx);
        });
    }

    bool is_convex_edge(size_t edge_idx) const {
        EDYN_ASSERT(edge_idx < m_is_convex_edge.size());
        return m_is_convex_edge[edge_idx];
    }

    bool is_boundary_edge(size_t edge_idx) const {
        EDYN_ASSERT(edge_idx < m_is_boundary_edge.size());
        return m_is_boundary_edge[edge_idx];
    }

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

    vector3 get_adjacent_face_normal(size_t tri_idx, size_t edge_idx) const {
        EDYN_ASSERT(tri_idx < m_adjacent_normals.size());
        EDYN_ASSERT(edge_idx < 3);
        return m_adjacent_normals[tri_idx][edge_idx];
    }

    bool has_per_vertex_friction() const;
    scalar get_vertex_friction(size_t vertex_idx) const;
    scalar get_edge_friction(size_t edge_idx, scalar fraction) const;
    scalar get_edge_friction(size_t edge_idx, vector3 point) const;
    scalar get_face_friction(size_t tri_idx, vector3 point) const;

    bool has_per_vertex_restitution() const;
    scalar get_vertex_restitution(size_t vertex_idx) const;
    scalar get_edge_restitution(size_t edge_idx, scalar fraction) const;
    scalar get_edge_restitution(size_t edge_idx, vector3 point) const;
    scalar get_face_restitution(size_t tri_idx, vector3 point) const;

    struct material_influence {
        material::id_type id;
        scalar fraction;
    };

    bool has_per_vertex_material_id() const;
    material::id_type get_vertex_material_id(size_t vertex_idx) const;
    std::array<material_influence, 2> get_edge_material_id(size_t edge_idx, scalar fraction) const;
    std::array<material_influence, 2> get_edge_material_id(size_t edge_idx, vector3 point) const;
    std::array<material_influence, 3> get_face_material_id(size_t tri_idx, vector3 point) const;

    /**
     * @brief Contact point will be ignored if they're deeper than this value.
     * This helps prevent objects from being pushed across the other side on
     * regions of the trimesh that have opposing triangles on the other side.
     * @return Current thickness.
     */
    scalar get_thickness() const { return m_thickness; }

    void set_thickness(scalar thickness) { m_thickness = thickness; }

    vector3 barycentric_coordinates(size_t tri_idx, vector3 point) const;
    scalar interpolate_triangle(size_t tri_idx, vector3 point, vector3 values) const;

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

    // Normal vector of adjacent faces which share an edge with the i-th face.
    std::vector<std::array<vector3, 3>> m_adjacent_normals;

    // Vertex indices for each unique edge. Each pair of values represent the
    // vertex indices for one edge.
    std::vector<unordered_pair<index_type>> m_edge_vertex_indices;

    // Indices of edges for each vertex. Each element is a list of indices of
    // edges that share the vertex.
    flat_nested_array<index_type> m_vertex_edge_indices;

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

    // Per-vertex friction and restitution coefficients.
    std::vector<scalar> m_friction;
    std::vector<scalar> m_restitution;
    std::vector<material::id_type> m_material_ids;

    scalar m_thickness {1};

    static_tree m_triangle_tree;
};

}

#endif // EDYN_SHAPES_TRIANGLE_MESH_HPP
