#include "edyn/shapes/triangle_mesh.hpp"
#include <limits>

namespace edyn {

void triangle_mesh::initialize() {
    // Order is important.
    build_tree();
    calculate_face_normals();
    init_edge_indices();
    init_vertex_edge_indices();
    init_face_edge_indices();
    calculate_edge_normals();
    calculate_concave_edges();
    calculate_concave_vertices();
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
    for (auto indices : m_indices) {
        for (size_t i = 0; i < 3; ++i) {
            auto j = (i + 1) % 3;
            auto i0 = indices[i];
            auto i1 = indices[j];
            auto contains = false;

            for (auto edge_indices : m_edge_indices) {
                if ((edge_indices[0] == i0 && edge_indices[1] == i1) ||
                    (edge_indices[0] == i1 && edge_indices[1] == i0)) {
                    contains = true;
                    break;
                }
            }

            if (!contains) {
                m_edge_indices.push_back({i0, i1});
            }
        }
    }
}

void triangle_mesh::init_vertex_edge_indices() {
    m_vertex_edge_index_ranges.reserve(m_vertices.size());

    for (index_type v_idx = 0; v_idx < m_vertices.size(); ++v_idx) {
        auto range_start = static_cast<index_type>(m_vertex_edge_indices.size());

        for (index_type e_idx = 0; e_idx < m_edge_indices.size(); ++e_idx) {
            auto edge_indices = m_edge_indices[e_idx];

            if (edge_indices[0] == v_idx || edge_indices[1] == v_idx) {
                m_vertex_edge_indices.push_back(e_idx);
            }
        }

        auto count = static_cast<index_type>(m_vertex_edge_indices.size() - range_start);
        EDYN_ASSERT(count > 0);
        m_vertex_edge_index_ranges.push_back({range_start, count});
    }
}

void triangle_mesh::init_face_edge_indices() {
    m_face_edge_indices.reserve(m_indices.size());
    m_edge_face_indices.resize(m_edge_indices.size());

    constexpr auto idx_max = std::numeric_limits<index_type>::max();
    std::fill(m_edge_face_indices.begin(), m_edge_face_indices.end(),
              std::array<index_type, 2>{idx_max, idx_max});

    for (index_type f_idx = 0; f_idx < m_indices.size(); ++f_idx) {
        auto indices = m_indices[f_idx];
        auto &face_edge_indices = m_face_edge_indices.emplace_back();

        for (size_t i = 0; i < 3; ++i) {
            auto j = (i + 1) % 3;
            auto i0 = indices[i];
            auto i1 = indices[j];
            auto e_idx = index_type{};

            for (; e_idx < m_edge_indices.size(); ++e_idx) {
                auto edge_indices = m_edge_indices[e_idx];

                if ((edge_indices[0] == i0 && edge_indices[1] == i1) ||
                    (edge_indices[0] == i1 && edge_indices[1] == i0)) {
                    // Assign edge index for i-th edge in this face.
                    face_edge_indices[i] = e_idx;
                    // Also assign a face index to this edge.
                    auto &edge_face_indices = m_edge_face_indices[e_idx];

                    if (edge_face_indices[0] == idx_max) {
                        edge_face_indices[0] = f_idx;
                    }

                    // Assign the same value for the second entry in case this
                    // is a boundary edge. If it's not, it will be replaced by
                    // the other face index later.
                    edge_face_indices[1] = f_idx;

                    break;
                }
            }

            // Ensure a value was assigned.
            EDYN_ASSERT(e_idx != m_edge_indices.size());
        }
    }

#ifdef EDYN_DEBUG
    for (auto edge_face_indices : m_edge_face_indices) {
        EDYN_ASSERT(edge_face_indices[0] != idx_max);
        EDYN_ASSERT(edge_face_indices[1] != idx_max);
    }
#endif
}

void triangle_mesh::calculate_edge_normals() {
    m_edge_normals.reserve(m_edge_indices.size());

    for (index_type e_idx = 0; e_idx < m_edge_indices.size(); ++e_idx) {
        auto vertex_indices = m_edge_indices[e_idx];
        auto face_indices = m_edge_face_indices[e_idx];
        auto edge_normals = m_edge_normals.emplace_back();

        for (size_t i = 0; i < face_indices.size(); ++i) {
            auto f_idx = face_indices[i];

            // If this edge is on the perimeter it will only have a single face
            // associated with it. Assign the edge normal from the first face
            // to the second entry to indicate a 180 degree convex edge.
            if (f_idx == std::numeric_limits<index_type>::max()) {
                EDYN_ASSERT(i > 0);
                edge_normals[i] = edge_normals[0];
                break;
            }

            for (size_t j = 0; j < 3; ++j) {
                auto k = (j + 1) % 3;
                auto i0 = m_indices[f_idx][j];
                auto i1 = m_indices[f_idx][k];

                if ((i0 == vertex_indices[0] && i1 == vertex_indices[1]) ||
                    (i0 == vertex_indices[1] && i1 == vertex_indices[0])) {
                    auto v0 = m_vertices[i0];
                    auto v1 = m_vertices[i1];
                    auto edge = v1 - v0;
                    auto normal = m_normals[f_idx];
                    edge_normals[i] = cross(normal, edge);
                }
            }
        }
    }
}

void triangle_mesh::calculate_concave_edges() {
    m_is_concave_edge.resize(m_edge_indices.size());

    for (size_t e_idx = 0; e_idx < m_edge_indices.size(); ++e_idx) {
        auto face_indices = m_edge_face_indices[e_idx];
        auto edge_normal0 = m_edge_normals[e_idx][0];
        auto face_normal1 = m_normals[face_indices[1]];
        m_is_concave_edge[e_idx] = dot(face_normal1, edge_normal0) > 0;
    }
}

void triangle_mesh::calculate_concave_vertices() {
    m_is_concave_vertex.resize(m_vertices.size());

    for (size_t v_idx = 0; v_idx < m_vertex_edge_index_ranges.size(); ++v_idx) {
        auto vertex_edge_index_range = m_vertex_edge_index_ranges[v_idx];
        auto first_vertex_edge_idx = vertex_edge_index_range[0];
        auto vertex_edge_count = vertex_edge_index_range[1];
        auto last_vertex_edge_idx = first_vertex_edge_idx + vertex_edge_count;
        m_is_concave_vertex[v_idx] = false;

        for (auto i = first_vertex_edge_idx; i < last_vertex_edge_idx; ++i) {
            auto edge_idx = m_vertex_edge_indices[i];
            if (m_is_concave_edge[edge_idx]) {
                m_is_concave_vertex[v_idx] = true;
                break;
            }
        }
    }
}

void triangle_mesh::build_tree() {
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
    m_tree.build(aabbs.begin(), aabbs.end(), report_leaf);
}

triangle_vertices triangle_mesh::get_triangle_vertices(size_t tri_idx) const {
    EDYN_ASSERT(tri_idx < m_indices.size());
    return {
        m_vertices[m_indices[tri_idx][0]],
        m_vertices[m_indices[tri_idx][1]],
        m_vertices[m_indices[tri_idx][2]]
    };
}

bool triangle_mesh::ignore_vertex(size_t vertex_idx, const vector3 &dir) const {
    if (m_is_concave_vertex[vertex_idx]) {
        return true;
    }

    auto v0 = m_vertices[vertex_idx];
    // `dir` must be within the pyramid that originates at `v0` and has the
    // edges sharing `v0` as face normals.
    auto vertex_edge_index_range = m_vertex_edge_index_ranges[vertex_idx];
    auto first_vertex_edge_idx = vertex_edge_index_range[0];
    auto vertex_edge_count = vertex_edge_index_range[1];
    auto last_vertex_edge_idx = first_vertex_edge_idx + vertex_edge_count;

    for (auto i = first_vertex_edge_idx; i < last_vertex_edge_idx; ++i) {
        auto edge_idx = m_vertex_edge_indices[i];
        auto vertex_indices = m_edge_indices[edge_idx];
        EDYN_ASSERT(vertex_indices[0] == vertex_idx || vertex_indices[1] == vertex_idx);
        auto other_vertex_idx = vertex_indices[0] != vertex_idx ? vertex_indices[0] : vertex_indices[1];
        EDYN_ASSERT(other_vertex_idx != vertex_idx);
        auto v1 = m_vertices[other_vertex_idx];
        auto edge = v1 - v0;

        if (dot(dir, edge) > 0) {
            return true;
        }
    }

    return false;
}

bool triangle_mesh::ignore_edge(size_t edge_idx, const vector3 &dir) const {
    if (m_is_concave_edge[edge_idx]) {
        return true;
    }

    auto normals = m_edge_normals[edge_idx];
    return dot(dir, normals[0]) > 0 || dot(dir, normals[1]) > 0;
}

}
