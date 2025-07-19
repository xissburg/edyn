#include "../common/common.hpp"
#include "edyn/parallel/job_dispatcher.hpp"
#include "edyn/shapes/create_paged_triangle_mesh.hpp"

TEST(test_paged_trimesh, voronoi_regions) {
    edyn::job_dispatcher::global().start(1);

    std::vector<edyn::vector3> vertices;
    std::vector<edyn::triangle_mesh::index_type> indices;

    vertices.push_back({1, 0, 1});
    vertices.push_back({1, 0, -1});
    vertices.push_back({-1, 0, -1});
    vertices.push_back({-1, 0, 1});
    vertices.push_back({0, 1, 0});
    vertices.push_back({2, 0, 0});

    indices.insert(indices.end(), {0, 1, 4});
    indices.insert(indices.end(), {1, 2, 4});
    indices.insert(indices.end(), {2, 3, 4});
    indices.insert(indices.end(), {3, 0, 4});
    indices.insert(indices.end(), {0, 5, 1});

    auto input = std::make_shared<edyn::paged_triangle_mesh_file_input_archive>("terrain_large.bin", &edyn::enqueue_task_default);
    auto trimesh = std::make_shared<edyn::paged_triangle_mesh>(std::static_pointer_cast<edyn::triangle_mesh_page_loader_base>(input));
    edyn::create_paged_triangle_mesh(*trimesh, vertices.begin(), vertices.end(), indices.begin(), indices.end(), 2, {}, {}, &edyn::enqueue_task_wait_default);

    auto offset = edyn::vector3_one * 0.01f;
    auto vertex_aabb = edyn::AABB{vertices[4] - offset, vertices[4] + offset};
    trimesh->visit_triangles(vertex_aabb, [&](size_t mesh_idx, size_t tri_idx) {
        auto submesh = trimesh->get_submesh(mesh_idx);
        auto tri_vertices = submesh->get_triangle_vertices(tri_idx);

        for (auto i = 0; i < 3; ++i) {
            auto &vertex = tri_vertices[i];
            if (vertex != vertices[4]) continue;

            auto vertex_idx = submesh->get_face_vertex_index(tri_idx, i);
            /* Not sure what to test lol */
        }
    });

    edyn::job_dispatcher::global().stop();
}
