#include "edyn/shapes/paged_triangle_mesh.hpp"
#include "edyn/parallel/parallel_for.hpp"
#include <atomic>
#include <limits>
#include <mutex>
#include <entt/entity/registry.hpp>

namespace edyn {

paged_triangle_mesh::paged_triangle_mesh(std::shared_ptr<triangle_mesh_page_loader_base> loader)
    : m_page_loader(loader)
{
    m_page_loader->on_load_sink().connect<&paged_triangle_mesh::assign_mesh>(*this);
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

void paged_triangle_mesh::assign_mesh(size_t index, std::shared_ptr<triangle_mesh> mesh) {
    // Use lock to prevent assigning to the same trimesh shared_ptr concurrently
    // if `unload_least_recently_visited_node` is executing in another thread.
    auto lock = std::lock_guard(m_lru_mutex);
    m_cache[index].trimesh = mesh;
    m_is_loading_submesh[index].store(false, std::memory_order_release);
}

bool paged_triangle_mesh::has_per_vertex_friction() const {
    for (auto &node : m_cache) {
        if (node.trimesh && node.trimesh->has_per_vertex_friction()) {
            return true;
        }
    }

    return false;
}

bool paged_triangle_mesh::has_per_vertex_restitution() const {
    for (auto &node : m_cache) {
        if (node.trimesh && node.trimesh->has_per_vertex_restitution()) {
            return true;
        }
    }

    return false;
}

}
