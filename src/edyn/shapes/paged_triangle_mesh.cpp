#include "edyn/shapes/paged_triangle_mesh.hpp"
#include "edyn/parallel/parallel_for.hpp"
#include "edyn/util/aabb_util.hpp"
#include <atomic>
#include <limits>
#include <entt/entt.hpp>
#include <mutex>

namespace edyn {

paged_triangle_mesh::paged_triangle_mesh(std::shared_ptr<triangle_mesh_page_loader_base> loader)
    : m_page_loader(loader)
{
    m_page_loader->on_load_delegate().connect<&paged_triangle_mesh::assign_mesh>(*this);
}

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
    EDYN_ASSERT(m_is_loading_submesh && trimesh_idx < m_cache.size());
    auto already_loading = m_is_loading_submesh[trimesh_idx].exchange(true, std::memory_order_relaxed);

    if (already_loading) {
        return;
    }

    auto &node = m_cache[trimesh_idx];

    if (node.trimesh) {
        m_is_loading_submesh[trimesh_idx].store(false, std::memory_order_relaxed);
        return;
    }

    EDYN_ASSERT(node.num_vertices < m_max_cache_num_vertices);
    // Load triangle mesh into cache. Clear cache if it would go
    // above limits.
    while (cache_num_vertices() + node.num_vertices > m_max_cache_num_vertices) {
        unload_least_recently_visited_node();
    }

    m_page_loader->load(trimesh_idx);
}

void paged_triangle_mesh::mark_recent_visit(size_t trimesh_idx) {
    auto lock = std::lock_guard(m_lru_mutex);
    auto it = std::find(m_lru_indices.begin(), m_lru_indices.end(), trimesh_idx);
    std::rotate(m_lru_indices.begin(), it, it + 1);
}

void paged_triangle_mesh::unload_least_recently_visited_node() {
    auto lock = std::lock_guard(m_lru_mutex);

    for (auto it = m_lru_indices.rbegin(); it != m_lru_indices.rend(); ++it) {
        auto &node = m_cache[*it];
        if (node.trimesh) {
            node.trimesh.reset();
            break;
        }
    }
}

void paged_triangle_mesh::calculate_edge_angles(scalar merge_distance) {
    // For each edge on the boundary of each trimesh, visit the triangles which
    // share that edge among all submeshes and adjust all edge normals and
    // vertex tangents to account
    size_t num_boundary_edges = 0;

    for (size_t i = 0; i < m_cache.size(); ++i) {
        num_boundary_edges += m_cache[i].trimesh->m_boundary_edge_indices.size();
    }

    parallel_for(size_t{0}, num_boundary_edges, [&] (size_t index) {
        size_t count = 0;
        size_t mesh_idx_i = 0;
        size_t boundary_edge_idx_i = 0;

        for (size_t i = 0; i < m_cache.size(); ++i) {
            auto num_boundary_edges_i = m_cache[i].trimesh->m_boundary_edge_indices.size();
            if (index >= count && index < count + num_boundary_edges_i) {
                mesh_idx_i = i;
                boundary_edge_idx_i = index - count;
                break;
            }
            count += num_boundary_edges_i;
        }

        auto &submesh_i = *m_cache[mesh_idx_i].trimesh;
        auto edge_idx = submesh_i.m_boundary_edge_indices[boundary_edge_idx_i];
        auto edge_vertices = submesh_i.get_edge_vertices(edge_idx);
        auto edge_aabb = AABB{
            min(edge_vertices[0], edge_vertices[1]),
            max(edge_vertices[0], edge_vertices[1]),
        };

        auto inset = vector3 {-merge_distance, -merge_distance, -merge_distance};
        auto visit_aabb = edge_aabb.inset(inset);

        this->visit_cache(visit_aabb, [&] (size_t mesh_idx_k, size_t tri_idx_k) {
            if (mesh_idx_i == mesh_idx_k) {
                return;
            }

            // Look for shared edge.
            const auto &vertices_k = this->get_triangle_vertices(mesh_idx_k, tri_idx_k);

            for (size_t i = 0; i < 3; ++i) {
                auto j = (i + 1) % 3;
                auto d0i = distance_sqr(edge_vertices[0], vertices_k[i]);
                auto d0j = distance_sqr(edge_vertices[0], vertices_k[j]);
                auto d1i = distance_sqr(edge_vertices[1], vertices_k[i]);
                auto d1j = distance_sqr(edge_vertices[1], vertices_k[j]);
                auto m = merge_distance * merge_distance;

                if ((d0i < m && d0j < m) || (d1i < m && d1j < m)) {
                    auto normal = this->get_submesh(mesh_idx_k)->get_triangle_normal(tri_idx_k);
                    auto edge = vertices_k[j] - vertices_k[i];
                    submesh_i.m_edge_normals[edge_idx][1] = normalize(cross(normal, edge));
                    break;
                }
            }
        });
    });
}

std::shared_ptr<triangle_mesh> paged_triangle_mesh::get_submesh(size_t idx) {
    return m_cache[idx].trimesh;
}

triangle_vertices paged_triangle_mesh::get_triangle_vertices(size_t mesh_idx, size_t tri_idx) {
    EDYN_ASSERT(mesh_idx < m_cache.size());
    EDYN_ASSERT(m_cache[mesh_idx].trimesh);
    return m_cache[mesh_idx].trimesh->get_triangle_vertices(tri_idx);
}

void paged_triangle_mesh::clear_cache() {
    for (auto &node : m_cache) {
        node.trimesh.reset();
    }
}

void paged_triangle_mesh::assign_mesh(size_t index, std::unique_ptr<triangle_mesh> mesh) {
    // Use lock to prevent assigning to the same trimesh shared_ptr concurrently
    // if `unload_least_recently_visited_node` is executing in another thread.
    auto lock = std::lock_guard(m_lru_mutex);
    m_cache[index].trimesh = std::move(mesh);
    m_is_loading_submesh[index].store(false, std::memory_order_release);
}

}
