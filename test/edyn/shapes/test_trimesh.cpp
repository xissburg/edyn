#include "../common/common.hpp"

TEST(test_trimesh, voronoi_regions) {
    auto vertices = std::vector<edyn::vector3>{};
    vertices.push_back({1, 0, 1});
    vertices.push_back({1, 0, -1});
    vertices.push_back({-1, 0, -1});
    vertices.push_back({-1, 0, 1});
    vertices.push_back({0, 1, 0});
    vertices.push_back({2, 0, 0});

    auto indices = std::vector<uint32_t>{};
    indices.insert(indices.end(), {0, 1, 4});
    indices.insert(indices.end(), {1, 2, 4});
    indices.insert(indices.end(), {2, 3, 4});
    indices.insert(indices.end(), {3, 0, 4});
    indices.insert(indices.end(), {0, 5, 1});

    auto trimesh = edyn::triangle_mesh{};
    trimesh.insert_vertices(vertices.begin(), vertices.end());
    trimesh.insert_indices(indices.begin(), indices.end());
    trimesh.initialize();

    ASSERT_EQ(trimesh.num_edges(), 10);

    ASSERT_FALSE(trimesh.is_convex_edge(trimesh.get_face_edge_index(4, 2)));
    ASSERT_TRUE(trimesh.is_convex_edge(trimesh.get_face_edge_index(0, 1)));
    ASSERT_TRUE(trimesh.is_convex_edge(trimesh.get_face_edge_index(2, 1)));

    ASSERT_TRUE(trimesh.is_boundary_edge(trimesh.get_face_edge_index(1, 0)));
    ASSERT_TRUE(trimesh.is_boundary_edge(trimesh.get_face_edge_index(2, 0)));
    ASSERT_TRUE(trimesh.is_boundary_edge(trimesh.get_face_edge_index(3, 0)));
    ASSERT_TRUE(trimesh.is_boundary_edge(trimesh.get_face_edge_index(4, 0)));
    ASSERT_TRUE(trimesh.is_boundary_edge(trimesh.get_face_edge_index(4, 1)));
    ASSERT_FALSE(trimesh.is_boundary_edge(trimesh.get_face_edge_index(4, 2)));

    ASSERT_VECTOR3_EQ(trimesh.get_adjacent_face_normal(0, 0), {0, 1, 0});

    ASSERT_VECTOR3_EQ(trimesh.get_aabb().min, {-1, 0, -1});
    ASSERT_VECTOR3_EQ(trimesh.get_aabb().max, {2, 1, 1});
}
