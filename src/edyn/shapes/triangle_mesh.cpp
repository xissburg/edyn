#include "edyn/shapes/triangle_mesh.hpp"
#include <limits>
#include <set>

namespace edyn {

void triangle_mesh::initialize() {
    // Order is important.
    calculate_face_normals();
    init_edge_indices();
    calculate_adjacent_normals();
    build_triangle_tree();
}

void triangle_mesh::calculate_face_normals() {
    m_normals.reserve(m_indices.size());

    for (auto indices : m_indices) {
        auto e0 = m_vertices[indices[1]] - m_vertices[indices[0]];
        auto e1 = m_vertices[indices[2]] - m_vertices[indices[1]];
        auto normal = normalize(cross(e0, e1));
        m_normals.push_back(normal);
    }
}

void triangle_mesh::init_edge_indices() {
    constexpr auto idx_max = std::numeric_limits<index_type>::max();
    m_face_edge_indices.resize(m_indices.size());
    auto vertex_edge_indices = std::vector<std::set<index_type>>(m_vertices.size());

    for (size_t face_idx = 0; face_idx < m_indices.size(); ++face_idx) {
        auto indices = m_indices[face_idx];

        for (size_t i = 0; i < 3; ++i) {
            auto j = (i + 1) % 3;
            auto i0 = indices[i];
            auto i1 = indices[j];
            auto pair = unordered_pair(i0, i1);
            auto edge_idx = SIZE_MAX;

            for (size_t k = 0; k < m_edge_vertex_indices.size(); ++k) {
                if (m_edge_vertex_indices[k] == pair) {
                    edge_idx = k;
                    break;
                }
            }

            if (edge_idx == SIZE_MAX) {
                edge_idx = m_edge_vertex_indices.size();
                m_edge_vertex_indices.push_back(pair);
                m_edge_face_indices.push_back({idx_max, idx_max});
            }

            vertex_edge_indices[i0].insert(edge_idx);
            vertex_edge_indices[i1].insert(edge_idx);

            m_face_edge_indices[face_idx][i] = edge_idx;

            auto &edge_face_indices = m_edge_face_indices[edge_idx];

            if (edge_face_indices[0] == idx_max) {
                edge_face_indices[0] = face_idx;
            } else if (face_idx != edge_face_indices[0]){
                edge_face_indices[1] = face_idx;
            }
        }
    }

    for (auto edge_indices : vertex_edge_indices) {
        m_vertex_edge_indices.push_array();

        for (auto edge_idx : edge_indices) {
            m_vertex_edge_indices.push_back(edge_idx);
        }
    }

    m_is_boundary_edge.resize(m_edge_vertex_indices.size());

    // Edges with a single valid _edge face index_ are at the boundary.
    for (index_type edge_idx = 0; edge_idx < m_edge_face_indices.size(); ++edge_idx) {
        auto &edge_face_indices = m_edge_face_indices[edge_idx];
        EDYN_ASSERT(edge_face_indices[0] != idx_max);

        auto is_boundary_edge = edge_face_indices[1] == idx_max;
        m_is_boundary_edge[edge_idx] = is_boundary_edge;

        if (is_boundary_edge) {
            // Assign the same value for the second entry for boundary edges,
            // thus defining a 180 degree convex edge.
            edge_face_indices[1] = edge_face_indices[0];
        }
    }
}

void triangle_mesh::calculate_adjacent_normals() {
    m_adjacent_normals.resize(m_indices.size());
    m_is_convex_edge.resize(m_edge_vertex_indices.size());

    for (size_t face_idx = 0; face_idx < m_indices.size(); ++face_idx) {
        for (size_t i = 0; i < 3; ++i) {
            auto edge_idx = m_face_edge_indices[face_idx][i];
            auto &edge_face_indices = m_edge_face_indices[edge_idx];
            auto other_face_idx = edge_face_indices[0] == face_idx ? edge_face_indices[1] : edge_face_indices[0];

            auto vertex_idx0 = m_indices[face_idx][i];
            auto vertex_idx1 = m_indices[face_idx][(i + 1) % 3];
            auto edge_dir = m_vertices[vertex_idx1] - m_vertices[vertex_idx0];
            auto edge_normal = cross(m_normals[face_idx], edge_dir);

            if (other_face_idx == face_idx) {
                // This is a boundary edge. Make adjacent normal point slightly
                // away in the edge direction to form a near 180 degree angle.
                m_adjacent_normals[face_idx][i] = -normalize(m_normals[face_idx] + edge_normal * 0.1);
                // Boundary edges are always convex.
                m_is_convex_edge[edge_idx] = true;
            } else {
                auto other_normal = m_normals[other_face_idx];
                m_adjacent_normals[face_idx][i] = other_normal;

                // Calculate edge convexity.
                auto is_convex = dot(other_normal, edge_normal) < -EDYN_EPSILON;
                m_is_convex_edge[edge_idx] = is_convex;
            }
        }
    }
}

void triangle_mesh::build_triangle_tree() {
    std::vector<AABB> aabbs;
    aabbs.reserve(num_triangles());

    for (size_t i = 0; i < num_triangles(); ++i) {
        auto verts = get_triangle_vertices(i);
        auto tri_aabb = get_triangle_aabb(verts);
        aabbs.push_back(tri_aabb);
    }

    auto report_leaf = [] (static_tree::tree_node &node, auto ids_begin, auto ids_end) {
        node.id = *ids_begin;
    };
    m_triangle_tree.build(aabbs.begin(), aabbs.end(), report_leaf);
}

triangle_vertices triangle_mesh::get_triangle_vertices(size_t tri_idx) const {
    EDYN_ASSERT(tri_idx < m_indices.size());
    auto indices = m_indices[tri_idx];
    return {
        m_vertices[indices[0]],
        m_vertices[indices[1]],
        m_vertices[indices[2]]
    };
}

}
