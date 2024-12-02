#ifndef EDYN_SHAPES_PAGED_TRIANGLE_MESH_HPP
#define EDYN_SHAPES_PAGED_TRIANGLE_MESH_HPP

#include <mutex>
#include <vector>
#include <atomic>
#include <memory>
#include "edyn/context/task.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/shapes/triangle_mesh.hpp"
#include "edyn/shapes/triangle_mesh_page_loader.hpp"

namespace edyn {

class paged_triangle_mesh_file_input_archive;
class paged_triangle_mesh_file_output_archive;
class finish_load_mesh_job;

// Forward declaration of `detail::submesh_builder` needed by `friend`
// declaration in `paged_triangle_mesh`.
namespace detail {
    struct submesh_builder;
}

/**
 * @brief A triangle mesh which loads chunks on demand.
 */
class paged_triangle_mesh {
public:
    struct triangle_mesh_node {
        size_t num_vertices;
        size_t num_indices;
        // Triangle mesh pointer. Will be nullptr if mesh is not loaded.
        std::shared_ptr<triangle_mesh> trimesh;
    };

    paged_triangle_mesh(std::shared_ptr<triangle_mesh_page_loader_base> loader);

    /**
     * @brief Visit submeshes which intersect the given AABB. It will start
     * loading submeshes that are not yet in the cache so subsequent calls
     * with a similar AABB should hit those submeshes.
     * @tparam Func Type of the function object to invoke.
     * @param aabb Query AABB.
     * @param func Will be called with submesh index.
     */
    template<typename Func>
    void visit_submeshes(const AABB &aabb, Func func) {
        m_tree.query(aabb, [&](auto tree_node_idx) {
            auto mesh_idx = m_tree.get_node(tree_node_idx).id;
            load_node_if_needed(mesh_idx);

            if (m_cache[mesh_idx].trimesh) {
                func(mesh_idx);
                mark_recent_visit(mesh_idx);
            }
        });
    }

    /**
     * @brief Loops over all edges present in the cache.
     * @tparam Func Type of the function object to invoke.
     * @param func Will be called with the submesh index and edge index.
     */
    template<typename Func>
    void visit_all_cached_edges(Func func) const {
        for (size_t mesh_idx = 0; mesh_idx < m_cache.size(); ++mesh_idx) {
            auto trimesh = m_cache[mesh_idx].trimesh;

            if (trimesh) {
                for (size_t edge_idx = 0; edge_idx < trimesh->num_edges(); ++edge_idx) {
                    func(mesh_idx, edge_idx);
                }
            }
        }
    }

    /**
     * @brief Visits all triangles that intersect the given AABB.
     * @param aabb The AABB to visit.
     * @param func An element into which the `operator()` will be called.
     *      The expected signature is:
     *      `void(uint32_t mesh_idx, uint32_t tri_idx)`
     *      Where:
     *      - `mesh_idx` is the index of the submesh.
     *      - `tri_idx` is the triangle index within the submesh.
     */
    template<typename Func>
    void visit_triangles(const AABB &aabb, Func func) {
        m_tree.query(aabb, [&](auto tree_node_idx) {
            auto mesh_idx = m_tree.get_node(tree_node_idx).id;
            load_node_if_needed(mesh_idx);
            auto trimesh = m_cache[mesh_idx].trimesh;

            if (trimesh) {
                trimesh->visit_triangles(aabb, [=](uint32_t tri_idx) {
                    func(mesh_idx, tri_idx);
                });
                mark_recent_visit(mesh_idx);
            }
        });
    }

    /**
     * @brief Visits all cached triangles that intersect the given AABB, which
     *      means no new triangle meshes will be loaded into the cache in the
     *      call.
     * @param aabb The AABB to visit.
     * @param func An element into which the `operator()` will be called.
     *      The expected signature is:
     *      `void(uint32_t mesh_idx, uint32_t tri_idx)`
     *      Where:
     *      - `mesh_idx` is the index of the submesh.
     *      - `tri_idx` is the triangle index within the submesh.
     */
    template<typename Func>
    void visit_cached_triangles(const AABB &aabb, Func func) const {
        for (size_t mesh_idx = 0; mesh_idx < m_cache.size(); ++mesh_idx) {
            auto trimesh = m_cache[mesh_idx].trimesh;

            if (trimesh) {
                trimesh->visit_triangles(aabb, [=](uint32_t tri_idx) {
                    func(mesh_idx, tri_idx);
                });
            }
        }
    }

    /**
     * @brief Visits all triangles of all cached nodes.
     * @param func An element into which the `operator()` will be called.
     *      The expected signature is:
     *      `void(uint32_t mesh_idx, uint32_t tri_idx)`
     *      Where:
     *      - `mesh_idx` is the index of the submesh.
     *      - `tri_idx` is the triangle index within the submesh.
     */
    template<typename Func>
    void visit_all_cached_triangles(Func func) const {
        for (size_t mesh_idx = 0; mesh_idx < m_cache.size(); ++mesh_idx) {
            auto trimesh = m_cache[mesh_idx].trimesh;

            if (trimesh) {
                for (size_t tri_idx = 0; tri_idx < trimesh->num_triangles(); ++tri_idx) {
                    func(mesh_idx, tri_idx);
                }
            }
        }
    }

    /**
     * @brief Performs a raycast on the submeshes and triangles. It will start
     * loading any submeshes that are touched by the segment so subsequent calls
     * will likely hit submeshes that are not currently in the cache.
     * @tparam Func Type of the function object to invoke.
     * @param p0 First point in the ray.
     * @param p1 Second point in the ray.
     * @param func Will be called with the submesh index and triangle index.
     */
    template<typename Func>
    void raycast(const vector3 &p0, const vector3 &p1, Func func) {
        m_tree.raycast(p0, p1, [&](auto tree_node_idx) {
            auto mesh_idx = m_tree.get_node(tree_node_idx).id;
            load_node_if_needed(mesh_idx);
            auto trimesh = m_cache[mesh_idx].trimesh;

            if (trimesh) {
                trimesh->raycast(p0, p1, [=](uint32_t tri_idx) {
                    func(mesh_idx, tri_idx);
                });
                mark_recent_visit(mesh_idx);
            }
        });
    }

    /**
     * @brief Performs a raycast on the submeshes and triangles. Only considers
     * submeshes that are present in the cache.
     * @tparam Func Type of the function object to invoke.
     * @param p0 First point in the ray.
     * @param p1 Second point in the ray.
     * @param func Will be called with the submesh index and triangle index.
     */
    template<typename Func>
    void raycast_cached(const vector3 &p0, const vector3 &p1, Func func) const {
        m_tree.raycast(p0, p1, [&](auto tree_node_idx) {
            auto mesh_idx = m_tree.get_node(tree_node_idx).id;
            auto trimesh = m_cache[mesh_idx].trimesh;

            if (trimesh) {
                trimesh->raycast(p0, p1, [=](uint32_t tri_idx) {
                    func(mesh_idx, tri_idx);
                });
            }
        });
    }

    /**
     * @brief Get AABB of entire mesh.
     * @return AABB of mesh.
     */
    AABB get_aabb() const {
        return m_tree.root_aabb();
    }

    /**
     * @brief Returns the number of vertices currently in the cache.
     * @return The size of the cache in number of vertices.
     */
    size_t cache_num_vertices() const;

    /**
     * @brief Get total number of sub-meshes this triangle mesh was
     * subdivided into.
     * @return Number of sub-meshes.
     */
    size_t num_submeshes() const {
        return m_cache.size();
    }

    /**
     * @brief Get a sub-mesh. Will be null if not loaded.
     * @param idx Sub-mesh index.
     * @return Pointer to submesh. Null if not loaded.
     */
    std::shared_ptr<triangle_mesh> get_submesh(size_t idx) {
        return m_cache[idx].trimesh;
    }

    /**
     * @brief Get position of vertices of a triangle in a submesh.
     * @param mesh_idx Sub-mesh index.
     * @param tri_idx Triangle index in the submesh.
     * @return Vertex positions.
     */
    triangle_vertices get_triangle_vertices(size_t mesh_idx, size_t tri_idx) const;

    void clear_cache();

    void assign_mesh(size_t index, std::shared_ptr<triangle_mesh>);

    auto & get_page_loader() {
        return *m_page_loader;
    }

    /**
     * @brief Check whether mesh has per-vertex friction.
     * @return Whether it has per-vertex friction.
     */
    bool has_per_vertex_friction() const;

    /**
     * @brief Check whether mesh has per-vertex restitution.
     * @return Whether it has per-vertex restitution.
     */
    bool has_per_vertex_restitution() const;

    /**
     * @brief Check whether mesh has per-vertex material IDs.
     * @return Whether it has per-vertex material IDs.
     */
    bool has_per_vertex_material_id() const;

    /**
     * @brief Get friction coefficient at a given point on the mesh surface.
     * @param point Point on mesh surface.
     * @return Friction coefficient.
     */
    scalar get_friction(vector3 point) const;

    /**
     * @brief Get restitution at a given point on the mesh surface.
     * @param point Point on mesh surface.
     * @return Coefficient of restitution.
     */
    scalar get_restitution(vector3 point) const;

    /**
     * @brief Get material IDs at a given point on the mesh surface.
     * @param point Point on mesh surface.
     * @return Material ID of each vertex of the triangle containing the given
     * point along with the influence of each material at that point.
     */
    std::array<triangle_mesh::material_influence, 3> get_material_id(vector3 point) const;

    scalar get_thickness() const { return m_thickness; }

    void set_thickness(scalar thickness);

    /**
     * @brief Maximum number of vertices in the cache. Before a new triangle mesh
     * is loaded, if the number of vertices would exceed this number, the
     * least recently visited nodes will be unloaded until the new total
     * number of vertices stays below this value.
     */
    size_t m_max_cache_num_vertices = 1 << 13;

    template<typename VertexIterator, typename IndexIterator>
    friend void create_paged_triangle_mesh(
            paged_triangle_mesh &paged_tri_mesh,
            VertexIterator vertex_begin, VertexIterator vertex_end,
            IndexIterator index_begin, IndexIterator index_end,
            size_t max_tri_per_submesh,
            const std::vector<vector3> &vertex_colors,
            vector3 color_scale,
            enqueue_task_wait_t *enqueue_task_wait);

    friend struct detail::submesh_builder;

    friend class paged_triangle_mesh_file_input_archive;
    friend class paged_triangle_mesh_file_output_archive;

    friend void serialize(paged_triangle_mesh_file_output_archive &archive,
                          paged_triangle_mesh &paged_tri_mesh);

    friend void serialize(paged_triangle_mesh_file_input_archive &archive,
                          paged_triangle_mesh &paged_tri_mesh);

private:
    void load_node_if_needed(size_t trimesh_idx);
    void mark_recent_visit(size_t trimesh_idx);
    void unload_least_recently_visited_node();

    static_tree m_tree;
    std::vector<triangle_mesh_node> m_cache;
    std::vector<size_t> m_lru_indices;
    std::mutex m_lru_mutex;
    std::unique_ptr<std::atomic<bool>[]> m_is_loading_submesh;
    std::shared_ptr<triangle_mesh_page_loader_base> m_page_loader;
    scalar m_thickness {1};
};

}

#endif // EDYN_SHAPES_PAGED_TRIANGLE_MESH_HPP
