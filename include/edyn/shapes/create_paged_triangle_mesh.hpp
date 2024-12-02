#ifndef EDYN_SHAPES_CREATE_PAGED_TRIANGLE_MESH_HPP
#define EDYN_SHAPES_CREATE_PAGED_TRIANGLE_MESH_HPP

#include <cstdint>
#include <entt/signal/delegate.hpp>
#include <memory>
#include "edyn/context/task.hpp"
#include "edyn/context/task_util.hpp"
#include "edyn/shapes/paged_triangle_mesh.hpp"
#include "edyn/context/settings.hpp"

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
               VertexIterator vertex_begin, IndexIterator index_begin,
               const std::vector<vector3> &vertex_colors,
               const vector3 &color_scale,
                enqueue_task_wait_t *enqueue_task_wait) {
        // Allocate space in cache for all submeshes.
        paged_tri_mesh.m_cache.resize(infos.size());

        // Create submeshes using the triangle indices stored in the `build_info`s.
        auto task_func = [&](unsigned start, unsigned end) {
            for (auto idx = start; idx < end; ++idx) {
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
                // `local_indices` maps local indices to global indices, i.e. the index
                // of the element is the vertex index in the submesh.
                auto local_indices = global_indices;
                std::sort(local_indices.begin(), local_indices.end());
                auto local_indices_erase_begin = std::unique(local_indices.begin(), local_indices.end());
                local_indices.erase(local_indices_erase_begin, local_indices.end());

                // Create triangle mesh for this leaf and allocate vertices and indices.
                auto submesh = std::make_unique<triangle_mesh>();
                submesh->m_vertices.reserve(local_indices.size());

                if (!vertex_colors.empty()) {
                    submesh->m_friction.reserve(local_indices.size());
                    submesh->m_restitution.reserve(local_indices.size());
                    submesh->m_material_ids.reserve(local_indices.size());
                }

                // Insert vertices into triangle mesh.
                for (auto idx : local_indices) {
                    submesh->m_vertices.push_back(*(vertex_begin + idx));

                    if (!vertex_colors.empty()) {
                        auto &color = vertex_colors[idx];
                        submesh->m_friction.push_back(color.x * color_scale.x);
                        submesh->m_restitution.push_back(color.y * color_scale.y);
                        auto mat_id = static_cast<material::id_type>(std::round(color.z * 255));
                        submesh->m_material_ids.push_back(mat_id);
                    }
                }

                submesh->m_indices.resize(local_num_triangles);
                submesh->m_normals.resize(local_num_triangles);
                submesh->m_adjacent_normals.resize(local_num_triangles);

                // Obtain local indices from global indices and add to triangle mesh.
                for (size_t tri_idx = 0; tri_idx < local_num_triangles; ++tri_idx) {
                    auto global_tri_idx = info.ids[tri_idx];

                    for (size_t i = 0; i < 3; ++i) {
                        auto global_vertex_idx = global_indices[tri_idx * 3 + i];
                        // The local vertex index is the index of the element in
                        // `local_indices` which is equals to `global_vertex_idx`.
                        auto it = std::find(local_indices.begin(), local_indices.end(), global_vertex_idx);
                        EDYN_ASSERT(it != local_indices.end());
                        auto local_vertex_idx = std::distance(local_indices.begin(), it);
                        submesh->m_indices[tri_idx][i] = local_vertex_idx;
                        // Assign adjacent normals as well.
                        submesh->m_adjacent_normals[tri_idx][i] = global_tri_mesh.m_adjacent_normals[global_tri_idx][i];
                    }

                    // Assign normals as well.
                    submesh->m_normals[tri_idx] = global_tri_mesh.m_normals[global_tri_idx];
                }

                // `initialize()` should not be called on the submesh. Initialize
                // submesh selectively, copying already calculated data from the
                // full triangle mesh, which includes adjacency information related
                // to the neighboring submeshes.
                submesh->init_edge_indices();
                submesh->build_triangle_tree();

                auto local_num_edges = submesh->m_edge_vertex_indices.size();
                submesh->m_is_convex_edge.resize(local_num_edges);

                // Assign edge normals.
                for (size_t edge_idx = 0; edge_idx < local_num_edges; ++edge_idx) {
                    // Find corresponding global edge index.
                    // Get indices of vertices of this edge in the submesh.
                    auto local_vertex_index0 = submesh->m_edge_vertex_indices[edge_idx].first;
                    auto local_vertex_index1 = submesh->m_edge_vertex_indices[edge_idx].second;
                    // Also get vertex indices in the global mesh.
                    auto global_vertex_index0 = local_indices[local_vertex_index0];
                    auto global_vertex_index1 = local_indices[local_vertex_index1];
                    // Get list of global edge indices which share the first vertex
                    // and find the edge that contains both global vertices.
                    auto global_edge_indices = global_tri_mesh.m_vertex_edge_indices[global_vertex_index0];

                #if EDYN_DEBUG && !EDYN_DISABLE_ASSERT
                    auto edge_was_found = false;
                #endif

                    for (size_t i = 0; i < global_edge_indices.size(); ++i) {
                        auto global_edge_idx = global_edge_indices[i];
                        auto global_edge_vertex_indices = global_tri_mesh.m_edge_vertex_indices[global_edge_idx];

                        // If one of the vertices is the second one then this must be it.
                        if (global_edge_vertex_indices[0] != global_vertex_index1 &&
                            global_edge_vertex_indices[1] != global_vertex_index1) {
                            continue;
                        }

                        // Ensure the first vertex index matches expectation.
                        EDYN_ASSERT(global_edge_vertex_indices[0] == global_vertex_index0 ||
                                    global_edge_vertex_indices[1] == global_vertex_index0);

                        auto is_convex = global_tri_mesh.m_is_convex_edge[global_edge_idx];
                        submesh->m_is_convex_edge[edge_idx] = is_convex;

                    #if EDYN_DEBUG && !EDYN_DISABLE_ASSERT
                        edge_was_found = true;
                    #endif

                        break;
                    }

                #if EDYN_DEBUG && !EDYN_DISABLE_ASSERT
                    EDYN_ASSERT(edge_was_found);
                #endif
                }

                // Create node.
                auto &paged_node = paged_tri_mesh.m_cache[idx];
                paged_node.num_vertices = submesh->m_vertices.size();
                paged_node.num_indices = submesh->m_indices.size();
                paged_node.trimesh = std::move(submesh);
            }
        };

        if (enqueue_task_wait) {
            auto task = task_delegate_t(entt::connect_arg_t<&decltype(task_func)::operator()>{}, task_func);
            (*enqueue_task_wait)(task, infos.size());
        } else {
            task_func(0, infos.size());
        }
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
 * @param vertex_colors Optional RGB vertex color values for per-vertex material.
 * R is friction coefficient, G is restitution, B is material ID.
 * @param color_scale Scaling factor to apply to normalized vertex colors.
 */
template<typename VertexIterator, typename IndexIterator>
void create_paged_triangle_mesh(
        paged_triangle_mesh &paged_tri_mesh,
        VertexIterator vertex_begin, VertexIterator vertex_end,
        IndexIterator index_begin, IndexIterator index_end,
        size_t max_tri_per_submesh,
        const std::vector<vector3> &vertex_colors,
        vector3 color_scale,
        enqueue_task_wait_t *enqueue_task_wait) {

    // Only allowed to create a mesh if this instance is empty.
    EDYN_ASSERT(paged_tri_mesh.m_tree.empty() && paged_tri_mesh.m_cache.empty());

    auto num_indices = static_cast<size_t>(std::distance(index_begin, index_end));
    auto num_triangles = num_indices / 3;

    // Create a `triangle_mesh` containing the full list of vertices and indices
    // and then break it up into smaller meshes.
    auto global_tri_mesh = triangle_mesh{};
    global_tri_mesh.insert_vertices(vertex_begin, vertex_end);
    global_tri_mesh.insert_indices(index_begin, index_end);
    global_tri_mesh.initialize();

    // Calculate AABB of each triangle.
    std::vector<AABB> aabbs(num_triangles);

    auto task_func = [&](unsigned start, unsigned end) {
        for (auto i = start; i < end; ++i) {
            auto verts = triangle_vertices{
                *(vertex_begin + *(index_begin + (i * 3 + 0))),
                *(vertex_begin + *(index_begin + (i * 3 + 1))),
                *(vertex_begin + *(index_begin + (i * 3 + 2)))
            };
            aabbs[i] = get_triangle_aabb(verts);
        }
    };

    if (enqueue_task_wait) {
        auto task = task_delegate_t(entt::connect_arg_t<&decltype(task_func)::operator()>{}, task_func);
        (*enqueue_task_wait)(task, num_triangles);
    } else {
        task_func(0, num_triangles);
    }

    // Build tree and submeshes.
    auto builder = detail::submesh_builder{};
    paged_tri_mesh.m_tree.build(aabbs.begin(), aabbs.end(), builder, max_tri_per_submesh);
    builder.build(paged_tri_mesh, global_tri_mesh, vertex_begin, index_begin, vertex_colors, color_scale, enqueue_task_wait);

    // Resize LRU queue to have the number of leaves.
    paged_tri_mesh.m_lru_indices.resize(paged_tri_mesh.m_cache.size());
    std::iota(paged_tri_mesh.m_lru_indices.begin(),
              paged_tri_mesh.m_lru_indices.end(), 0);

    paged_tri_mesh.m_is_loading_submesh = std::make_unique<std::atomic<bool>[]>(paged_tri_mesh.m_cache.size());
}

}

#endif // EDYN_SHAPES_CREATE_PAGED_TRIANGLE_MESH_HPP
