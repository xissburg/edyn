#ifndef EDYN_SHAPES_PAGED_TRIANGLE_MESH_HPP
#define EDYN_SHAPES_PAGED_TRIANGLE_MESH_HPP

#include <mutex>
#include <vector>
#include <atomic>
#include <memory>
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
    template<typename VertexIterator, typename IndexIterator, typename IdIterator>
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
     * @brief Visits all triangles that intersect the given AABB.
     * @param aabb The AABB to visit.
     * @param func An element into which the `operator()` will be called.
     *      The expected signature is: 
     *      `void(uint32_t trimesh_idx, uint32_t tri_idx)`
     *      Where:
     *      - `trimesh_idx` is the index of the submesh.
     *      - `tri_idx` is the triangle index within the submesh.
     */
    template<typename Func>
    void visit(const AABB &aabb, Func func) {
        constexpr auto inset = vector3 {
            -contact_breaking_threshold, 
            -contact_breaking_threshold, 
            -contact_breaking_threshold
        };
        auto inset_aabb = aabb.inset(inset);
        
        m_tree.visit(inset_aabb, [&] (auto trimesh_idx) {
            load_node_if_needed(trimesh_idx);
            auto trimesh = m_cache[trimesh_idx].trimesh;

            if (trimesh) {
                trimesh->visit(inset_aabb, [=] (uint32_t tri_idx) {
                    func(trimesh_idx, tri_idx);
                });
                mark_recent_visit(trimesh_idx);
            }
        });
    }

    /**
     * @brief Visits all triangles of all nodes.
     * @param func An element into which the `operator()` will be called.
     *      The expected signature is: 
     *      `void(uint32_t trimesh_idx, uint32_t tri_idx)`
     *      Where:
     *      - `trimesh_idx` is the index of the submesh.
     *      - `tri_idx` is the triangle index within the submesh.
     */
    template<typename Func>
    void visit_all(Func func) {        
        for (size_t i = 0; i < m_cache.size(); ++i) {
            load_node_if_needed(i);
            auto trimesh = m_cache[i].trimesh;

            if (trimesh) {
                for (size_t j = 0; j < trimesh->num_triangles(); ++j) {
                    func(i, j);
                }
            }
        }
    }

    /**
     * @brief Visits all cached triangles that intersect the given AABB, which
     *      means no new triangle meshes will be loaded into the cache in the 
     *      call.
     * @param aabb The AABB to visit.
     * @param func An element into which the `operator()` will be called.
     *      The expected signature is: 
     *      `void(uint32_t trimesh_idx, uint32_t tri_idx)`
     *      Where:
     *      - `trimesh_idx` is the index of the submesh.
     *      - `tri_idx` is the triangle index within the submesh.
     */
    template<typename Func>
    void visit_cache(const AABB &aabb, Func func) const {
        for (size_t mesh_idx = 0; mesh_idx < m_cache.size(); ++mesh_idx) {
            auto trimesh = m_cache[mesh_idx].trimesh;

            if (trimesh) {
                trimesh->visit(aabb, [=] (uint32_t tri_idx) {
                    func(mesh_idx, tri_idx);
                });
            }
        }
    }

    /**
     * @brief Visits all triangles of all cached nodes.
     * @param func An element into which the `operator()` will be called.
     *      The expected signature is: 
     *      `void(uint32_t trimesh_idx, uint32_t tri_idx)`
     *      Where:
     *      - `trimesh_idx` is the index of the submesh.
     *      - `tri_idx` is the triangle index within the submesh.
     */
    template<typename Func>
    void visit_cache_all(Func func) const {
        for (size_t mesh_idx = 0; mesh_idx < m_cache.size(); ++mesh_idx) {
            auto trimesh = m_cache[mesh_idx].trimesh;

            if (trimesh) {
                for (size_t tri_idx = 0; tri_idx < trimesh->num_triangles(); ++tri_idx) {
                    func(mesh_idx, tri_idx);
                }
            }
        }
    }

    AABB get_aabb() const {
        return m_tree.root_aabb();
    }

    /**
     * @brief Returns the number of vertices currently in the cache.
     * @return The size of the cache in number of vertices.
     */
    size_t cache_num_vertices() const;

    std::shared_ptr<triangle_mesh> get_submesh(size_t idx);

    triangle_vertices get_triangle_vertices(size_t mesh_idx, size_t tri_idx);

    void clear_cache();

    void assign_mesh(size_t index, std::unique_ptr<triangle_mesh>);

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
        size_t max_tri_per_submesh);

    template<typename VertexIterator, typename IndexIterator, typename IdIterator>
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
    void calculate_edge_angles(scalar merge_distance);

    static_tree m_tree;
    std::vector<triangle_mesh_node> m_cache;
    std::vector<size_t> m_lru_indices;
    std::mutex m_lru_mutex;
    std::unique_ptr<std::atomic<bool>[]> m_is_loading_submesh;
    std::shared_ptr<triangle_mesh_page_loader_base> m_page_loader;
};

}

#endif // EDYN_SHAPES_PAGED_TRIANGLE_MESH_HPP