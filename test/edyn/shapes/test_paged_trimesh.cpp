#include "../common/common.hpp"
#include "edyn/shapes/create_paged_triangle_mesh.hpp"
#include "edyn/shapes/paged_triangle_mesh.hpp"

class triangle_mesh_page_loader: public edyn::triangle_mesh_page_loader_base {
public:
    void load(size_t index) override {}
    virtual entt::delegate<loaded_mesh_func_t> & on_load_delegate() override {
        return m_loaded_delegate;
    }

private:
    entt::delegate<loaded_mesh_func_t> m_loaded_delegate;
};

TEST(test_paged_trimesh, voronoi_regions) {
    edyn::init();

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

    auto loader = std::make_shared<triangle_mesh_page_loader>();
    auto trimesh = edyn::paged_triangle_mesh(loader);
    edyn::create_paged_triangle_mesh(trimesh, vertices.begin(), vertices.end(), indices.begin(), indices.end(), 2);

    auto offset = edyn::vector3_one * 0.01f;
    auto vertex_aabb = edyn::AABB{vertices[4] - offset, vertices[4] + offset};
    trimesh.visit(vertex_aabb, [&] (size_t mesh_idx, size_t tri_idx) {
        auto submesh = trimesh.get_submesh(mesh_idx);
        auto tri_vertices = submesh->get_triangle_vertices(tri_idx);

        for (auto i = 0; i < 3; ++i) {
            auto &vertex = tri_vertices[i];
            if (vertex != vertices[4]) continue;

            auto vertex_idx = submesh->get_face_vertex_index(tri_idx, i);
            ASSERT_TRUE(submesh->in_vertex_voronoi(vertex_idx, {0, 1, 0}));
            ASSERT_TRUE(submesh->in_vertex_voronoi(vertex_idx, edyn::rotate(edyn::quaternion_axis_angle({0, 0, 1}, edyn::pi / 4 - EDYN_EPSILON), {0, 1, 0})));
            ASSERT_FALSE(submesh->in_vertex_voronoi(vertex_idx, {0, -1, 0}));
            ASSERT_FALSE(submesh->in_vertex_voronoi(vertex_idx, {1, 0, 0.5}));
        }
    });
}
