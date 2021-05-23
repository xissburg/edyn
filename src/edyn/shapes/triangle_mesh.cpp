#include "edyn/shapes/triangle_mesh.hpp"
#include "edyn/math/scalar.hpp"
#include <limits>
#include <cmath>

namespace edyn {

void triangle_mesh::initialize() {
    // Order is important.
    calculate_face_normals();
    init_edge_indices();
    init_vertex_tangents();
    init_face_edge_indices();
    calculate_edge_normals();
    calculate_concave_edges();
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

void triangle_mesh::init_vertex_tangents() {
    m_vertex_tangent_ranges.reserve(m_vertices.size());

    // The total number of vertex tangents is not easily calculated. Since each
    // vertex is shared by at least 2 edges, reserve the number of vertices
    // times 2.
    m_vertex_tangents.reserve(m_vertices.size() * 2);

    for (index_type v_idx = 0; v_idx < m_vertices.size(); ++v_idx) {
        auto range_start = static_cast<index_type>(m_vertex_tangents.size());
        auto range_end = range_start;

        for (index_type e_idx = 0; e_idx < m_edge_indices.size(); ++e_idx) {
            auto edge_indices = m_edge_indices[e_idx];

            if (edge_indices[0] == v_idx || edge_indices[1] == v_idx) {
                auto v0 = m_vertices[edge_indices[0]];
                auto v1 = m_vertices[edge_indices[1]];
                auto tangent = normalize(v1 - v0);

                if (edge_indices[1] == v_idx) {
                    tangent *= -1; // Point away of vertex.
                }

                m_vertex_tangents.push_back(tangent);
                ++range_end;
            }
        }

        EDYN_ASSERT(range_start != range_end);
        m_vertex_tangent_ranges.push_back({range_start, range_end});
    }
}

void triangle_mesh::init_face_edge_indices() {
    m_face_edge_indices.reserve(m_indices.size());
    m_edge_face_indices.resize(m_edge_indices.size());

    constexpr auto idx_max = std::numeric_limits<index_type>::max();
    std::fill(m_edge_face_indices.begin(), m_edge_face_indices.end(),
              std::array<index_type, 2>{idx_max, idx_max});

    m_is_boundary_edge.resize(m_edge_indices.size());

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
                    } else {
                        EDYN_ASSERT(edge_face_indices[0] != idx_max);
                        edge_face_indices[1] = f_idx;
                    }

                    break;
                }
            }

            // Ensure a value was assigned.
            EDYN_ASSERT(e_idx != m_edge_indices.size());
        }
    }

    // Edges with a single valid _edge face index_ are at the boundary.
    for (index_type edge_idx = 0; edge_idx < m_edge_face_indices.size(); ++edge_idx) {
        auto &edge_face_indices = m_edge_face_indices[edge_idx];
        EDYN_ASSERT(edge_face_indices[0] != idx_max);

        auto is_boundary_edge = edge_face_indices[1] == idx_max;
        m_is_boundary_edge[edge_idx] = is_boundary_edge;

        if (is_boundary_edge) {
            m_is_boundary_edge[edge_idx] = true;
            // Assign the same value for the second entry for boundary edges,
            // thus defining a 180 degree convex edge.
            edge_face_indices[1] = edge_face_indices[0];
        }
    }
}

void triangle_mesh::calculate_edge_normals() {
    m_edge_normals.reserve(m_edge_indices.size());

    for (index_type e_idx = 0; e_idx < m_edge_indices.size(); ++e_idx) {
        auto &vertex_indices = m_edge_indices[e_idx];
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

void triangle_mesh::calculate_concave_edges() {
    m_is_concave_edge.resize(m_edge_indices.size());
    m_is_concave_vertex.resize(m_vertices.size());
    std::fill(m_is_concave_vertex.begin(), m_is_concave_vertex.end(), false);

    for (size_t e_idx = 0; e_idx < m_edge_indices.size(); ++e_idx) {
        // Boundary edges are always convex.
        if (is_boundary_edge(e_idx)) {
            continue;
        }

        auto face_indices = m_edge_face_indices[e_idx];
        auto edge_normal0 = m_edge_normals[e_idx][0];
        auto face_normal1 = m_normals[face_indices[1]];

        auto is_concave = dot(face_normal1, edge_normal0) > 0;
        m_is_concave_edge[e_idx] = is_concave;

        if (is_concave) {
            // If an edge is concave, both of its vertices are also concave.
            auto vertex_indices = m_edge_indices[e_idx];
            m_is_concave_vertex[vertex_indices[0]] = true;
            m_is_concave_vertex[vertex_indices[1]] = true;
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

std::array<vector3, 2> triangle_mesh::get_convex_edge_face_normals(size_t edge_idx) const {
    EDYN_ASSERT(!is_concave_edge(edge_idx));

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
    /* if (m_is_concave_vertex[vertex_idx]) {
        return false;
    } */

    // `dir` must be within the pyramid that originates at the vertex and has
    // the edges sharing this vertex as face normals.
    auto vertex_tangent_range = m_vertex_tangent_ranges[vertex_idx];
    auto first_tangent_idx = vertex_tangent_range[0];
    auto last_tangent_idx = vertex_tangent_range[1];

    for (auto i = first_tangent_idx; i < last_tangent_idx; ++i) {
        auto tangent = m_vertex_tangents[i];

        if (dot(dir, tangent) > EDYN_EPSILON) {
            return false;
        }
    }

    return true;
}

bool triangle_mesh::in_edge_voronoi(size_t edge_idx, const vector3 &dir) const {
    if (m_is_concave_edge[edge_idx]) {
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
