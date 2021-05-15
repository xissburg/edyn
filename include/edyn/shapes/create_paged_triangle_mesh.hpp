#ifndef EDYN_SHAPES_CREATE_PAGED_TRIANGLE_MESH_HPP
#define EDYN_SHAPES_CREATE_PAGED_TRIANGLE_MESH_HPP

#include <cstdint>
#include <memory>
#include "edyn/shapes/paged_triangle_mesh.hpp"
#include "edyn/parallel/parallel_for.hpp"

namespace edyn {

namespace detail {
/**
 * `submesh_builder` is passed as the `report_leaf` argument to `static_tree::build`
 * in `create_paged_triangle_mesh`. On each invocation of `operator()` it stores the
 * ids for that submesh. After `static_tree::build` finishes, the `build` function
 * must be called to create the submeshes and add them to the paged triangle mesh.
 */
template<typename VertexIterator, typename IndexIterator, typename IdIterator>
struct submesh_builder {
    paged_triangle_mesh *paged_tri_mesh;
    VertexIterator vertex_begin, vertex_end;
    IndexIterator index_begin, index_end;

    struct build_info {
        std::vector<uint32_t> ids;
    };
    std::vector<build_info> infos;

    void operator()(static_tree::tree_node &node, IdIterator ids_begin, IdIterator ids_end) {
        // Assign the `build_info` index as the node id which will later be
        // matched with the submesh id.
        node.id = infos.size();
        // Setup new `build_info` with the triangle ids for this submesh.
        auto info = build_info();
        info.ids.resize(std::distance(ids_begin, ids_end));
        std::copy(ids_begin, ids_end, info.ids.begin());
        infos.push_back(info);
    }

    void build() {
        // Allocate space in cache for all submeshes.
        paged_tri_mesh->m_cache.resize(infos.size());

        // Create submeshes using the triangle indices stored in the `build_info`s.
        parallel_for(size_t{0}, infos.size(), [&] (size_t idx) {
            auto &info = infos[idx];

            // Transform triangle indices into vertex indices.
            auto local_num_triangles = info.ids.size();
            auto global_indices = std::vector<size_t>();
            global_indices.reserve(local_num_triangles * 3);

            for (auto it = info.ids.begin(); it != info.ids.end(); ++it) {
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
            trimesh->m_vertices.reserve(local_indices.size());

            // Insert vertices into triangle mesh.
            for (auto idx : local_indices) {
                trimesh->m_vertices.push_back(*(vertex_begin + idx));
            }

            trimesh->m_indices.resize(local_num_triangles);

            // Obtain local indices from global indices and add to triangle mesh.
            for (auto tri_idx = 0; tri_idx < local_num_triangles; ++tri_idx) {
                for (size_t i = 0; i < 3; ++i) {
                    auto v_idx = global_indices[tri_idx * 3 + i];
                    auto it = std::find(local_indices.begin(), local_indices.end(), v_idx);
                    auto local_idx = std::distance(local_indices.begin(), it);
                    trimesh->m_indices[tri_idx][i] = local_idx;
                }
            }

            // Initialize triangle mesh.
            trimesh->build_tree();

            // Edge-angles are calculated after the entire tree is ready so that
            // neighboring triangles that reside in another submesh are also
            // considered. Thus, only allocate space for edge angle info.
            trimesh->initialize();

            // Create node.
            auto &paged_node = paged_tri_mesh->m_cache[idx];
            paged_node.num_vertices = trimesh->m_vertices.size();
            paged_node.num_indices = trimesh->m_indices.size();
            paged_node.trimesh = std::move(trimesh);
        });
    }
};
} // namespace detail

/**
 * Creates a paged triangle mesh from a list of vertices and indices.
 * @tparam VertexIterator Vertex iterator type.
 * @tparam IndexIterator Index iterator type.
 * @param vertex_begin Begin iterator for the vertex list.
 * @param vertex_end End iterator for the vertex list.
 * @param index_begin Begin iterator for the index list.
 * @param index_end End iterator for the index list.
 * @param max_tri_per_submesh Maximum number of triangles for submeshes.
 */
template<typename VertexIterator, typename IndexIterator>
void create_paged_triangle_mesh(
        paged_triangle_mesh &paged_tri_mesh,
        VertexIterator vertex_begin, VertexIterator vertex_end,
        IndexIterator index_begin, IndexIterator index_end,
        size_t max_tri_per_submesh) {

    // Only allowed to create a mesh if this instance is empty.
    EDYN_ASSERT(paged_tri_mesh.m_tree.empty() && paged_tri_mesh.m_cache.empty());

    // Do not limit cache size when building the triangle mesh. Keep all submeshes
    // in memory to later calculate edge angles (adjacency).
    auto original_max_cache_size = paged_tri_mesh.m_max_cache_num_vertices;
    paged_tri_mesh.m_max_cache_num_vertices = SIZE_MAX;

    auto num_indices = static_cast<size_t>(std::distance(index_begin, index_end));
    auto num_triangles = num_indices / 3;

    // Calculate AABB of each triangle.
    std::vector<AABB> aabbs(num_triangles);

    parallel_for(size_t{0}, num_triangles, [&] (size_t i) {
        auto verts = triangle_vertices{
            *(vertex_begin + *(index_begin + (i * 3 + 0))),
            *(vertex_begin + *(index_begin + (i * 3 + 1))),
            *(vertex_begin + *(index_begin + (i * 3 + 2)))
        };
        aabbs[i] = get_triangle_aabb(verts);
    });

    // Build tree and submeshes.
    auto builder = detail::submesh_builder<VertexIterator, IndexIterator, std::vector<uint32_t>::iterator>{
        &paged_tri_mesh, vertex_begin, vertex_end, index_begin, index_end
    };
    paged_tri_mesh.m_tree.build(aabbs.begin(), aabbs.end(), builder, max_tri_per_submesh);
    builder.build();

    // Resize LRU queue to have the number of leaves.
    paged_tri_mesh.m_lru_indices.resize(paged_tri_mesh.m_cache.size());
    std::iota(paged_tri_mesh.m_lru_indices.begin(),
              paged_tri_mesh.m_lru_indices.end(), 0);

    paged_tri_mesh.m_is_loading_submesh = std::make_unique<std::atomic<bool>[]>(paged_tri_mesh.m_cache.size());

    // Calculate edge angles.
    constexpr scalar merge_distance = 0.01;
    paged_tri_mesh.calculate_edge_angles(merge_distance);

    // Reset cache settings.
    paged_tri_mesh.m_max_cache_num_vertices = original_max_cache_size;
}

}

#endif // EDYN_SHAPES_CREATE_PAGED_TRIANGLE_MESH_HPP
