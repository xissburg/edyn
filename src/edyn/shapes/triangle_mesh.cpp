#include "edyn/shapes/triangle_mesh.hpp"
#include <limits>
#include <set>

namespace edyn {

void triangle_mesh::initialize() {
    // Order is important.
    calculate_face_normals();
    init_edge_indices();
    init_vertex_tangents();
    calculate_edge_normals();
    calculate_convex_edges();
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
            auto pair = commutative_pair(i0, i1);
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

void triangle_mesh::init_vertex_tangents() {
    // The total number of vertex tangents is not easily calculated. Since each
    // vertex is shared by at least 2 edges, reserve the number of vertices
    // times 2.
    m_vertex_tangents.reserve_data(m_vertices.size() * 2);
    m_vertex_tangents.reserve_nested(m_vertices.size());

    for (size_t v_idx = 0; v_idx < m_vertices.size(); ++v_idx) {
        m_vertex_tangents.push_array();
        auto edge_indices = m_vertex_edge_indices[v_idx];

        for (size_t k = 0; k < edge_indices.size(); ++k) {
            auto edge_idx = edge_indices[k];
            auto i0 = m_edge_vertex_indices[edge_idx].first;
            auto i1 = m_edge_vertex_indices[edge_idx].second;
            auto v0 = m_vertices[i0];
            auto v1 = m_vertices[i1];
            auto tangent = normalize(v1 - v0);

            if (i1 == v_idx) {
                tangent *= -1; // Point away of vertex.
            }

            m_vertex_tangents.push_back(tangent);
        }
    }
}

void triangle_mesh::calculate_edge_normals() {
    m_edge_normals.reserve(m_edge_vertex_indices.size());

    for (index_type e_idx = 0; e_idx < m_edge_vertex_indices.size(); ++e_idx) {
        auto &vertex_indices = m_edge_vertex_indices[e_idx];
        auto &face_indices = m_edge_face_indices[e_idx];
        auto &edge_normals = m_edge_normals.emplace_back();

        for (size_t i = 0; i < face_indices.size(); ++i) {
            auto f_idx = face_indices[i];

            // If this edge is on the perimeter it will only have a single face
            // associated with it. Assign the edge normal from the first face
            // plus an offset to the second entry to indicate a near 180 degree
            // convex edge.
            if (i > 0 && f_idx == face_indices[0]) {
                auto normal = m_normals[f_idx];
                edge_normals[i] = edge_normals[0] - normal * 0.001;
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
                    edge_normals[i] = normalize(cross(normal, edge));
                }
            }
        }
    }
}

void triangle_mesh::calculate_convex_edges() {
    m_is_convex_edge.resize(m_edge_vertex_indices.size());

    for (size_t e_idx = 0; e_idx < m_edge_vertex_indices.size(); ++e_idx) {
        if (is_boundary_edge(e_idx)) {
            // Boundary edges are always convex.
            m_is_convex_edge[e_idx] = true;
            continue;
        }

        auto face_indices = m_edge_face_indices[e_idx];
        auto edge_normal0 = m_edge_normals[e_idx][0];
        auto face_normal1 = m_normals[face_indices[1]];

        auto is_convex = dot(face_normal1, edge_normal0) < -EDYN_EPSILON;
        m_is_convex_edge[e_idx] = is_convex;
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

std::array<vector3, 2> triangle_mesh::get_convex_edge_face_normals(size_t edge_idx) const {
    EDYN_ASSERT(is_convex_edge(edge_idx));

    auto edge_vertices = get_edge_vertices(edge_idx);
    auto edge_dir = edge_vertices[1] - edge_vertices[0];
    auto edge_normals = get_edge_normals(edge_idx);
    auto face_normals = std::array<vector3, 2>{
        normalize(cross(edge_dir, edge_normals[0])),
        normalize(cross(edge_dir, edge_normals[1]))
    };

    // Face normals could be pointing in the wrong direction. Since this is
    // a convex edge, the dot product of the normal of one face with the
    // edge normal of the other must be smaller than zero.
    face_normals[0] *= std::copysign(scalar(1), -dot(face_normals[0], edge_normals[1]));
    face_normals[1] *= std::copysign(scalar(1), -dot(face_normals[1], edge_normals[0]));

    return face_normals;
}

bool triangle_mesh::in_vertex_voronoi(size_t vertex_idx, const vector3 &dir) const {
    // `dir` must be within the pyramid that originates at the vertex and has
    // the edges sharing this vertex as face normals.
    auto tangents = m_vertex_tangents[vertex_idx];

    for (size_t i = 0; i < tangents.size(); ++i) {
        if (dot(dir, tangents[i]) > EDYN_EPSILON) {
            return false;
        }
    }

    return true;
}

bool triangle_mesh::in_edge_voronoi(size_t edge_idx, const vector3 &dir) const {
    if (!m_is_convex_edge[edge_idx]) {
        return false;
    }

    auto normals = m_edge_normals[edge_idx];
    return dot(dir, normals[0]) < EDYN_EPSILON && dot(dir, normals[1]) < EDYN_EPSILON;
}

bool triangle_mesh::ignore_triangle_feature(size_t tri_idx, triangle_feature tri_feature,
                                            size_t feature_idx, const vector3 &dir) const {
    switch (tri_feature) {
    case triangle_feature::edge:
        return !in_edge_voronoi(get_face_edge_index(tri_idx, feature_idx), dir);
    case triangle_feature::vertex:
        return !in_vertex_voronoi(get_face_vertex_index(tri_idx, feature_idx), dir);
    case triangle_feature::face:
        return dot(dir, get_triangle_normal(tri_idx)) < 0.9;
    }
    return false;
}

}
