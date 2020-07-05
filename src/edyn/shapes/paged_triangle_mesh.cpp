#include "edyn/shapes/paged_triangle_mesh.hpp"

namespace edyn {

size_t paged_triangle_mesh::cache_num_vertices() const {
    size_t count = 0;

    for (auto &node : m_cache) {
        if (!node.trimesh) {
            continue;
        }

        count += node.num_vertices;
    }

    return count;
}

void paged_triangle_mesh::load_node_if_needed(size_t trimesh_idx) {
    auto &node = m_cache[trimesh_idx];

    if (!node.trimesh && !m_is_loading_submesh[trimesh_idx]) {
        EDYN_ASSERT(node.num_vertices < m_max_cache_num_vertices);
        // Load triangle mesh into cache. Clear cache if it would go
        // above limits.
        while (cache_num_vertices() + node.num_vertices > m_max_cache_num_vertices) {
            unload_least_recently_visited_node();
        }

        m_is_loading_submesh[trimesh_idx] = true;
        m_page_loader->load(trimesh_idx);
    }
}

void paged_triangle_mesh::mark_recent_visit(size_t trimesh_idx) {
    auto it = std::find(m_lru_indices.begin(), m_lru_indices.end(), trimesh_idx);
    std::rotate(m_lru_indices.begin(), it, it + 1);
}

void paged_triangle_mesh::unload_least_recently_visited_node() {
    for (auto it = m_lru_indices.rbegin(); it != m_lru_indices.rend(); ++it) {
        auto &node = m_cache[*it];
        if (node.trimesh) {
            unload_node(node);
            break;
        }
    }
}

void paged_triangle_mesh::unload_node(triangle_mesh_node &node) {
    node.trimesh.reset();
}

void paged_triangle_mesh::calculate_edge_angles(scalar merge_distance) {
    // For each triangle, visit surrounding triangles (which could be in a
    // separate triangle mesh) and find shared vertices which form an edge
    // and calculate the angle between them.
    size_t num_triangles = 0;

    for (size_t i = 0; i < m_cache.size(); ++i) {
        num_triangles += m_cache[i].trimesh->num_triangles();
    }

    parallel_for(size_t{0}, num_triangles, [&] (size_t index) {
        size_t count = 0;
        size_t mesh_idx_i = 0;
        size_t tri_idx_i = 0;

        for (size_t i = 0; i < m_cache.size(); ++i) {
            auto num_triangles_i = m_cache[i].trimesh->num_triangles();
            if (index >= count && index < count + num_triangles_i) {
                mesh_idx_i = i;
                tri_idx_i = index - count;
                break;
            }
            count += num_triangles_i;
        }

        auto &submesh_i = m_cache[mesh_idx_i].trimesh;

        auto vertices_i = triangle_vertices{
            submesh_i->vertices[submesh_i->indices[tri_idx_i * 3 + 0]],
            submesh_i->vertices[submesh_i->indices[tri_idx_i * 3 + 1]],
            submesh_i->vertices[submesh_i->indices[tri_idx_i * 3 + 2]]
        };

        // Normal vector of i-th triangle.
        auto edge0_i = vertices_i[1] - vertices_i[0];
        auto edge1_i = vertices_i[2] - vertices_i[1];
        auto normal_i = cross(edge0_i, edge1_i);
        auto normal_len_sqr_i = length_sqr(normal_i);

        if (normal_len_sqr_i > EDYN_EPSILON) {
            normal_i /= std::sqrt(normal_len_sqr_i);
        }

        auto inset = vector3 {-merge_distance, -merge_distance, -merge_distance};
        auto tri_aabb = get_triangle_aabb(vertices_i).inset(inset);

        this->visit_cache(tri_aabb, [&] (size_t mesh_idx_k, size_t tri_idx_k, 
                                         const triangle_vertices &vertices_k) {
            if (mesh_idx_i == mesh_idx_k && tri_idx_i == tri_idx_k) {
                return;
            }

            // Look for shared edge.
            std::pair<size_t, size_t> shared_idx[2];
            auto num_shared_vertices = 0;

            for (size_t m = 0; m < 3; ++m) {
                for (size_t n = 0; n < 3; ++n) {
                    if (distance_sqr(vertices_i[m], vertices_k[n]) < merge_distance * merge_distance) {
                        shared_idx[num_shared_vertices] = std::make_pair(m, n);
                        ++num_shared_vertices;

                        if (num_shared_vertices > 1) {
                            break;
                        }
                    }
                }
            }

            if (num_shared_vertices < 2) {
                return;
            }

            auto vertex0_idx_i = shared_idx[0].first;
            auto vertex1_idx_i = shared_idx[1].first;
            size_t edge_idx_i;

            /*...*/if ((vertex0_idx_i == 0 && vertex1_idx_i == 1) ||
                        (vertex0_idx_i == 1 && vertex1_idx_i == 0)) {
                edge_idx_i = 0;
            } else if ((vertex0_idx_i == 1 && vertex1_idx_i == 2) ||
                        (vertex0_idx_i == 2 && vertex1_idx_i == 1)) {
                edge_idx_i = 1;
            } else {
                edge_idx_i = 2;
            } 

            // Find index of the vertex in triangle k not in shared edge.
            auto vertex0_idx_k = shared_idx[0].second;
            auto vertex1_idx_k = shared_idx[1].second;
            size_t vertex2_idx_k;

            /*...*/if ((vertex0_idx_k == 0 && vertex1_idx_k == 1) ||
                        (vertex0_idx_k == 1 && vertex1_idx_k == 0)) {
                vertex2_idx_k = 2;
            } else if ((vertex0_idx_k == 1 && vertex1_idx_k == 2) ||
                        (vertex0_idx_k == 2 && vertex1_idx_k == 1)) {
                vertex2_idx_k = 0;
            } else {
                vertex2_idx_k = 1;
            }

            // Check if the vertex in triangle k which is not in the shared 
            // edge is in front of or behind the plane of triangle i.
            auto concave = dot(normal_i, vertices_k[vertex2_idx_k] - vertices_k[vertex0_idx_k]) > -EDYN_EPSILON;
            submesh_i->is_concave_edge[tri_idx_i * 3 + edge_idx_i] = concave;

            // Normal vector of k-th triangle.
            auto edge0_k = vertices_k[1] - vertices_k[0];
            auto edge1_k = vertices_k[2] - vertices_k[1];
            auto normal_k = cross(edge0_k, edge1_k);
            auto normal_len_sqr_k = length_sqr(normal_k);

            if (normal_len_sqr_k > EDYN_EPSILON) {
                normal_k /= std::sqrt(normal_len_sqr_k);
            }

            auto cos_angle = dot(normal_i, normal_k);
            submesh_i->cos_angles[tri_idx_i * 3 + edge_idx_i] = cos_angle;
        });
    });
}

triangle_mesh *paged_triangle_mesh::get_submesh(size_t idx) {
    return m_cache[idx].trimesh.get();
}

void paged_triangle_mesh::clear_cache() {
    for (auto &node : m_cache) {
        node.trimesh.reset();
    }
}

void paged_triangle_mesh::assign_mesh(size_t index, std::unique_ptr<triangle_mesh> &mesh) {
    m_cache[index].trimesh = std::move(mesh);
    m_is_loading_submesh[index] = false;
}

}