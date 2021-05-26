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
struct submesh_builder {
    struct build_info {
        std::vector<uint32_t> ids;
    };
    std::vector<build_info> infos;

    template<typename It>
    void operator()(static_tree::tree_node &node, It ids_begin, It ids_end) {
        // Assign the `build_info` index as the node id which will later be
        // matched with the submesh id.
        node.id = infos.size();
        // Setup new `build_info` with the triangle ids for this submesh.
        auto info = build_info();
        info.ids.resize(std::distance(ids_begin, ids_end));
        std::copy(ids_begin, ids_end, info.ids.begin());
        infos.push_back(info);
    }

    template<typename VertexIterator, typename IndexIterator>
    void build(paged_triangle_mesh &paged_tri_mesh, const triangle_mesh &global_tri_mesh,
               VertexIterator vertex_begin, VertexIterator vertex_end,
               IndexIterator index_begin, IndexIterator index_end) {
        // Allocate space in cache for all submeshes.
        paged_tri_mesh.m_cache.resize(infos.size());

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
            auto submesh = std::make_unique<triangle_mesh>();
            submesh->m_vertices.reserve(local_indices.size());

            // Insert vertices into triangle mesh.
            for (auto idx : local_indices) {
                submesh->m_vertices.push_back(*(vertex_begin + idx));
            }

            submesh->m_indices.resize(local_num_triangles);

            // Obtain local indices from global indices and add to triangle mesh.
            for (auto tri_idx = 0; tri_idx < local_num_triangles; ++tri_idx) {
                for (size_t i = 0; i < 3; ++i) {
                    auto vertex_idx = global_indices[tri_idx * 3 + i];
                    // The local vertex index is the index of the element in
                    // `local_indices` which is equals to the global `vertex_idx`.
                    auto it = std::find(local_indices.begin(), local_indices.end(), vertex_idx);
                    auto local_vertex_idx = std::distance(local_indices.begin(), it);
                    submesh->m_indices[tri_idx][i] = local_vertex_idx;
                }

                auto global_tri_idx = info.ids[tri_idx];
                submesh->m_normals[tri_idx] = global_tri_mesh.m_normals[global_tri_idx];
            }

            // `initialize()` should not be called on the submesh. Initialize
            // submesh selectively, copying already calculated data from the
            // full triangle mesh, which includes adjacency information related
            // to the neighboring submeshes.
            submesh->init_edge_indices();
            submesh->init_face_edge_indices();
            submesh->build_triangle_tree();

            submesh->m_vertex_tangent_ranges.reserve(submesh->m_vertices.size());
            submesh->m_vertex_tangents.reserve(submesh->m_vertices.size() * 2);

            // Assign vertex tangents.
            for (auto local_vertex_idx = 0; local_vertex_idx < local_indices.size(); ++local_vertex_idx) {
                auto global_vertex_idx = local_indices[local_vertex_idx];
                auto global_range = global_tri_mesh.m_vertex_tangent_ranges[global_vertex_idx];
                auto global_range_start = global_range[0];
                auto global_range_end = global_range[1];

                auto local_range_start = static_cast<triangle_mesh::index_type>(submesh->m_vertex_tangents.size());
                auto local_range_end = local_range_start;

                for (auto i = global_range_start; i < global_range_end; ++i) {
                    auto tangent = global_tri_mesh.m_vertex_tangents[i];
                    submesh->m_vertex_tangents.push_back(tangent);
                    ++local_range_end;
                }

                EDYN_ASSERT(local_range_start != local_range_end);
                submesh->m_vertex_tangent_ranges.push_back({local_range_start, local_range_end});
            }

            submesh->m_edge_normals.resize(submesh->m_edge_indices.size());
            submesh->m_is_convex_edge.resize(submesh->m_edge_indices.size());

            // Assign edge normals.
            for (auto edge_idx = 0; edge_idx < submesh->m_edge_indices.size(); ++edge_idx) {
                size_t global_vertex_indices[] = {
                    local_indices[submesh->m_edge_indices[edge_idx][0]],
                    local_indices[submesh->m_edge_indices[edge_idx][1]]
                };
                for (auto global_edge_idx = 0; global_edge_idx < global_tri_mesh.m_edge_indices.size(); ++global_edge_idx) {
                    auto indices = global_tri_mesh.m_edge_indices[global_edge_idx];
                    auto is_same = indices[0] == global_vertex_indices[0] &&
                                   indices[1] == global_vertex_indices[1];
                    auto is_reversed = indices[0] == global_vertex_indices[1] &&
                                       indices[1] == global_vertex_indices[0];

                    if (is_same || is_reversed) {
                        submesh->m_edge_normals[edge_idx] = global_tri_mesh.m_edge_normals[global_edge_idx];

                        if (is_reversed) {
                            std::swap(submesh->m_edge_normals[edge_idx][0], submesh->m_edge_normals[edge_idx][1]);
                        }

                        submesh->m_is_convex_edge[edge_idx] = global_tri_mesh.m_is_convex_edge[global_edge_idx];
                    }
                }
            }

            // Create node.
            auto &paged_node = paged_tri_mesh.m_cache[idx];
            paged_node.num_vertices = submesh->m_vertices.size();
            paged_node.num_indices = submesh->m_indices.size();
            paged_node.trimesh = std::move(submesh);
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

    auto num_vertices = std::distance(vertex_begin, vertex_end);
    auto num_indices = static_cast<size_t>(std::distance(index_begin, index_end));
    auto num_triangles = num_indices / 3;

    // Create a `triangle_mesh` containing the full list of vertices and indices
    // and then break it up into smaller meshes.
    auto global_tri_mesh = triangle_mesh{};
    global_tri_mesh.m_vertices.reserve(num_vertices);
    global_tri_mesh.m_indices.reserve(num_triangles);

    global_tri_mesh.m_vertices.insert(global_tri_mesh.m_vertices.end(), vertex_begin, vertex_end);

    for (auto it = index_begin; it != index_end; it += 3) {
        global_tri_mesh.m_indices.push_back({*it, *(it + 1), *(it + 2)});
    }

    global_tri_mesh.initialize();

    // Do not limit cache size when building the triangle mesh. Keep all submeshes
    // in memory to later calculate edge angles (adjacency).
    auto original_max_cache_size = paged_tri_mesh.m_max_cache_num_vertices;
    paged_tri_mesh.m_max_cache_num_vertices = SIZE_MAX;

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
    auto builder = detail::submesh_builder{};
    paged_tri_mesh.m_tree.build(aabbs.begin(), aabbs.end(), builder, max_tri_per_submesh);
    builder.build(paged_tri_mesh, global_tri_mesh, vertex_begin, vertex_end, index_begin, index_end);

    // Resize LRU queue to have the number of leaves.
    paged_tri_mesh.m_lru_indices.resize(paged_tri_mesh.m_cache.size());
    std::iota(paged_tri_mesh.m_lru_indices.begin(),
              paged_tri_mesh.m_lru_indices.end(), 0);

    paged_tri_mesh.m_is_loading_submesh = std::make_unique<std::atomic<bool>[]>(paged_tri_mesh.m_cache.size());

    // Reset cache settings.
    paged_tri_mesh.m_max_cache_num_vertices = original_max_cache_size;
}

}

#endif // EDYN_SHAPES_CREATE_PAGED_TRIANGLE_MESH_HPP
