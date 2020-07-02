#ifndef EDYN_SHAPES_PAGED_TRIANGLE_MESH_HPP
#define EDYN_SHAPES_PAGED_TRIANGLE_MESH_HPP

#include <map>
#include <vector>
#include <memory>
#include <iterator>
#include <entt/entt.hpp>
#include "edyn/math/constants.hpp"
#include "edyn/shapes/triangle_mesh.hpp"
#include "edyn/shapes/triangle_mesh_page_loader.hpp"
#include "edyn/parallel/parallel_for.hpp"

namespace edyn {

// Forward declaration of `detail::submesh_builder` needed by `friend` 
// declaration in `paged_triangle_mesh`.
namespace detail {
    template<typename VertexIterator, typename IndexIterator, typename IdIterator>
    struct submesh_builder;
}

class paged_triangle_mesh {
public:
    struct triangle_mesh_node {
        size_t num_vertices;
        size_t num_indices;
        // Triangle mesh unique pointer. Will be nullptr if mesh is not loaded.
        std::unique_ptr<triangle_mesh> trimesh;
    };

    paged_triangle_mesh(std::shared_ptr<triangle_mesh_page_loader_base> loader)
        : m_page_loader(loader)
    {}

    /**
     * @brief Visits all triangles that intersect the given AABB.
     * @param aabb The AABB to visit.
     * @param func An element into which the `operator()` will be called.
     *      The expected signature is: 
     *      `void(uint32_t trimesh_idx, uint32_t tri_idx, const triangle_vertices &vertices)`
     *      Where:
     *      - `trimesh_idx` is the index of the submesh.
     *      - `tri_idx` is the triangle index within the submesh.
     *      - `vertices` are the positions of the triangle vertices.
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
            auto &node = m_cache[trimesh_idx];

            if (!node.trimesh) {
                // Load triangle mesh into cache. Clear cache if it would go
                // above limits.
                while (cache_num_vertices() + node.num_vertices > m_max_cache_num_vertices) {
                    unload_least_recently_visited_node();
                }

                node.trimesh = std::make_unique<triangle_mesh>();
                m_page_loader->load(trimesh_idx, *node.trimesh);
            }

            EDYN_ASSERT(node.trimesh);

            node.trimesh->visit(inset_aabb, [=] (uint32_t tri_idx, const triangle_vertices &vertices) {
                func(trimesh_idx, tri_idx, vertices);
            });
            mark_recent_visit(trimesh_idx);
        });
    }

    /**
     * @brief Visits all triangles of all nodes.
     * @param func An element into which the `operator()` will be called.
     *      The expected signature is: 
     *      `void(uint32_t trimesh_idx, uint32_t tri_idx, const triangle_vertices &vertices)`
     *      Where:
     *      - `trimesh_idx` is the index of the submesh.
     *      - `tri_idx` is the triangle index within the submesh.
     *      - `vertices` are the positions of the triangle vertices.
     */
    template<typename Func>
    void visit_all(Func func) {        
        for (size_t i = 0; i < m_cache.size(); ++i) {
            auto &node = m_cache[i];

            if (!node.trimesh) {
                // Load triangle mesh into cache. Clear cache if it would go
                // above limits.
                while (cache_num_vertices() + node.num_vertices > m_max_cache_num_vertices) {
                    unload_least_recently_visited_node();
                }

                node.trimesh = std::make_unique<triangle_mesh>();
                m_page_loader->load(i, *node.trimesh);
            }

            EDYN_ASSERT(node.trimesh);

            node.trimesh->visit_all([=] (uint32_t tri_idx, const triangle_vertices &vertices) {
                func(i, tri_idx, vertices);
            });
        }
    }

    /**
     * @brief Visits all cached triangles that intersect the given AABB, which
     *      means no new triangle meshes will be loaded into the cache in the 
     *      call.
     * @param aabb The AABB to visit.
     * @param func An element into which the `operator()` will be called.
     *      The expected signature is: 
     *      `void(uint32_t trimesh_idx, uint32_t tri_idx, const triangle_vertices &vertices)`
     *      Where:
     *      - `trimesh_idx` is the index of the submesh.
     *      - `tri_idx` is the triangle index within the submesh.
     *      - `vertices` are the positions of the triangle vertices.
     */
    template<typename Func>
    void visit_cache(const AABB &aabb, Func func) const {
        constexpr auto inset = vector3 {
            -contact_breaking_threshold, 
            -contact_breaking_threshold, 
            -contact_breaking_threshold
        };

        for (size_t i = 0; i < m_cache.size(); ++i) {
            auto &node = m_cache[i];

            if (node.trimesh) {
                node.trimesh->visit(aabb, [=] (uint32_t tri_idx, const triangle_vertices &vertices) {
                    func(i, tri_idx, vertices);
                });
            }
        }
    }

    /**
     * @brief Visits all cached triangles of all nodes.
     * @param func An element into which the `operator()` will be called.
     *      The expected signature is: 
     *      `void(uint32_t trimesh_idx, uint32_t tri_idx, const triangle_vertices &vertices)`
     *      Where:
     *      - `trimesh_idx` is the index of the submesh.
     *      - `tri_idx` is the triangle index within the submesh.
     *      - `vertices` are the positions of the triangle vertices.
     */
    template<typename Func>
    void visit_cache_all(Func func) const {
        for (size_t i = 0; i < m_cache.size(); ++i) {
            auto &node = m_cache[i];

            if (node.trimesh) {
                node.trimesh->visit_all( [=] (uint32_t tri_idx, const triangle_vertices &vertices) {
                    func(i, tri_idx, vertices);
                });
            }
        }
    }

    const AABB &get_aabb() const {
        EDYN_ASSERT(!m_tree.m_nodes.empty());
        return m_tree.m_nodes.front().aabb;
    }

    /**
     * @brief Returns the number of vertices currently in the cache.
     * @return The size of the cache in number of vertices.
     */
    size_t cache_num_vertices() const;

    triangle_mesh *get_submesh(size_t idx);

    void clear_cache();

    /**
     * @brief Maximum number of vertices in the cache. Before a new triangle mesh
     *      is loaded, if the number of vertices would exceed this number, the 
     *      least recently visited nodes will be unloaded until the new total 
     *      number of vertices stays below this value.
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

    friend class paged_triangle_mesh_memory_input_archive;
    friend class paged_triangle_mesh_memory_output_archive;

private:
    void mark_recent_visit(size_t trimesh_idx);
    void unload_least_recently_visited_node();
    void unload_node(triangle_mesh_node &node);
    void calculate_edge_angles(scalar merge_distance);

    static_tree m_tree;
    std::vector<triangle_mesh_node> m_cache;
    std::vector<size_t> m_lru_indices;
    std::shared_ptr<triangle_mesh_page_loader_base> m_page_loader;
};

}

#endif // EDYN_SHAPES_PAGED_TRIANGLE_MESH_HPP