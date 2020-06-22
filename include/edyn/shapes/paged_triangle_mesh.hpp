#ifndef EDYN_SHAPES_PAGED_TRIANGLE_MESH_HPP
#define EDYN_SHAPES_PAGED_TRIANGLE_MESH_HPP

#include <map>
#include <vector>
#include <memory>
#include <entt/entt.hpp>
#include "edyn/math/constants.hpp"
#include "edyn/shapes/triangle_mesh.hpp"
#include "edyn/shapes/triangle_mesh_page_loader.hpp"
#include "edyn/parallel/job_dispatcher.hpp"

namespace edyn {

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
        return m_aabb;
    }

    /**
     * @brief Returns the number of vertices currently in the cache.
     * @return The size of the cache in number of vertices.
     */
    size_t cache_num_vertices() const;

    triangle_mesh *get_submesh(size_t idx);

    template<typename VertexIterator, typename IndexIterator, typename OutputArchiveSource>
    void load(VertexIterator vertex_begin, VertexIterator vertex_end,
              IndexIterator index_begin, IndexIterator index_end,
              OutputArchiveSource &output_archive_source,
              uint32_t max_obj_per_leaf) {
        // Do not limit cache size when building the triangle mesh. Keep all submeshes
        // in memory to later calculate edge angles (adjacency).
        auto original_max_cache_size = m_max_cache_num_vertices;
        m_max_cache_num_vertices = SIZE_MAX;

        calculate_aabb(vertex_begin, vertex_end);

        auto num_indices = std::distance(index_begin, index_end);
        auto num_triangles = num_indices / 3;
        
        // Calculate AABB of all triangles.
        std::vector<AABB> aabbs;
        aabbs.reserve(num_triangles);

        for (size_t i = 0; i < num_triangles; ++i) {
            auto verts = triangle_vertices{
                *(vertex_begin + *(index_begin + (i * 3 + 0))),
                *(vertex_begin + *(index_begin + (i * 3 + 1))),
                *(vertex_begin + *(index_begin + (i * 3 + 2)))
            };

            auto tri_aabb = get_triangle_aabb(verts);
            aabbs.push_back(tri_aabb);
        }

        auto report_leaf = [&] (static_tree::tree_node &node, auto ids_begin, auto ids_end) {
            // Transform triangle indices into vertex indices.
            auto local_num_triangles = std::distance(ids_begin, ids_end);
            auto global_indices = std::vector<size_t>();
            global_indices.reserve(local_num_triangles * 3);
            
            for (auto it = ids_begin; it != ids_end; ++it) {
                for (size_t i = 0; i < 3; ++i) {
                    auto index = *(index_begin + ((*it) * 3 + i));
                    global_indices.push_back(index);
                }
            }

            // Transform global indices into local indices by removing duplicates.
            // `local_indices` maps local indices to global indices.
            auto local_indices = global_indices;
            std::sort(local_indices.begin(), local_indices.end());
            auto local_indices_erase_begin = std::unique(local_indices.begin(), local_indices.end());
            local_indices.erase(local_indices_erase_begin, local_indices.end());

            // Create triangle mesh for this leaf and allocate vertices and indices.
            auto trimesh = std::make_unique<triangle_mesh>();
            trimesh->vertices.reserve(local_indices.size());
            trimesh->indices.reserve(global_indices.size());

            // Insert vertices into triangle mesh.
            for (auto idx : local_indices) {
                trimesh->vertices.push_back(*(vertex_begin + idx));
            }

            // Obtain local indices from global indices and add to triangle mesh.
            for (auto idx : global_indices) {
                auto it = std::find(local_indices.begin(), local_indices.end(), idx);
                auto local_idx = std::distance(local_indices.begin(), it);
                trimesh->indices.push_back(local_idx);
            }

            // Initialize triangle mesh.
            trimesh->calculate_aabb();
            trimesh->build_tree();

            // Edge-angles are calculated after the entire tree is ready so that
            // neighboring triangles that reside in another submesh are also 
            // considered. Thus, only allocate space for edge angle info.
            trimesh->initialize_edge_angles();

            // Create node.
            node.id = m_cache.size();
            auto &paged_node = m_cache.emplace_back();
            paged_node.num_vertices = trimesh->vertices.size();
            paged_node.num_indices = trimesh->indices.size();
            paged_node.trimesh = std::move(trimesh);
        };

        m_tree.build(aabbs.begin(), aabbs.end(), report_leaf, max_obj_per_leaf);

        // Resize LRU queue to have the number of leaves.
        m_lru_indices.resize(m_cache.size());
        std::iota(m_lru_indices.begin(), m_lru_indices.end(), 0);
        
        // Calculate edge angles.
        constexpr scalar merge_distance = 0.01;
        calculate_edge_angles(merge_distance);

        // Serialize all triangle meshes.
        for (size_t i = 0; i < m_cache.size(); ++i) {
            auto output = output_archive_source(i);
            auto &node = m_cache[i];
            serialize(output, *node.trimesh);
        }

        // Unload all triangle meshes.
        for (auto &node : m_cache) {
            node.trimesh.reset();
        }

        m_max_cache_num_vertices = original_max_cache_size;
    }

    /**
     * @brief Maximum number of vertices in the cache. Before a new triangle mesh
     *      is loaded, if the number of vertices would exceed this number, the 
     *      least recently visited nodes will be unloaded until the new total 
     *      number of vertices stays below this value.
     */
    size_t m_max_cache_num_vertices = 1 << 13;

private:
    void mark_recent_visit(size_t trimesh_idx);
    void unload_least_recently_visited_node();
    void unload_node(triangle_mesh_node &node);
    void calculate_edge_angles(scalar merge_distance);

    template<typename VertexIterator>
    void calculate_aabb(VertexIterator vertex_begin, VertexIterator vertex_end) {
        m_aabb.min = vector3_max;
        m_aabb.max = -vector3_max;

        for (auto it = vertex_begin; it != vertex_end; ++it) {
            m_aabb.min = min(m_aabb.min, *it);
            m_aabb.max = max(m_aabb.max, *it);
        }
    }

    AABB m_aabb;
    static_tree m_tree;
    std::vector<triangle_mesh_node> m_cache;
    std::vector<size_t> m_lru_indices;
    std::shared_ptr<triangle_mesh_page_loader_base> m_page_loader;
};

}

#endif // EDYN_SHAPES_PAGED_TRIANGLE_MESH_HPP