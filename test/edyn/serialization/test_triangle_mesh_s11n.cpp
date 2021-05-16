#include "../common/common.hpp"

TEST(triangle_mesh_serialization, test) {
    // Create triangle mesh.
    auto trimesh = edyn::triangle_mesh();

    auto extent_x = 2;
    auto extent_z = 12;
    auto num_vertices_x = 4;
    auto num_vertices_z = 24;
    std::vector<edyn::vector3> vertices;
    std::vector<edyn::triangle_mesh::index_type> indices;
    edyn::make_plane_mesh(extent_x, extent_z, num_vertices_x, num_vertices_z,
                          vertices, indices);

    trimesh.m_vertices = std::move(vertices);
    trimesh.m_indices.resize(indices.size() / 3);

    for (size_t i = 0; i < trimesh.m_indices.size(); ++i) {
        for (size_t j = 0; j < 3; ++j) {
            trimesh.m_indices[i][j] = indices[i * 3 + j];
        }
    }

    for (int z = 0; z < num_vertices_z; ++z) {
        auto t = (edyn::scalar(z) / edyn::scalar(num_vertices_z - 1)) * 2 - 1;
        auto y = (t*t*t*t - t*t) * 1.2;

        for (int x = 0; x < num_vertices_x; ++x) {
            trimesh.m_vertices[z * num_vertices_x + x].y = y + (x == 0 || x == num_vertices_x - 1 ? 0.1 : 0);
        }
    }

    trimesh.initialize();

    auto filename = "trimesh.bin";

    { // Do it in its own scope so the file gets closed on destruction.
        auto output = edyn::file_output_archive(filename);
        edyn::serialize(output, trimesh);
    }

    auto input_trimesh = edyn::triangle_mesh();

    {
        auto input = edyn::file_input_archive(filename);
        edyn::serialize(input, input_trimesh);
    }

    // Check sizes.
    ASSERT_EQ(trimesh.m_vertices.size()       , input_trimesh.m_vertices.size());
    ASSERT_EQ(trimesh.m_indices.size()        , input_trimesh.m_indices.size());
    ASSERT_EQ(trimesh.m_is_concave_edge.size(), input_trimesh.m_is_concave_edge.size());
    ASSERT_EQ(trimesh.m_triangle_tree.m_nodes.size()   , input_trimesh.m_triangle_tree.m_nodes.size());

    // Check AABBs.
    ASSERT_SCALAR_EQ(trimesh.get_aabb().min.y, input_trimesh.get_aabb().min.y);
    ASSERT_SCALAR_EQ(trimesh.get_aabb().min.x, input_trimesh.get_aabb().min.x);
    ASSERT_SCALAR_EQ(trimesh.get_aabb().min.z, input_trimesh.get_aabb().min.z);
    ASSERT_SCALAR_EQ(trimesh.get_aabb().max.x, input_trimesh.get_aabb().max.x);
    ASSERT_SCALAR_EQ(trimesh.get_aabb().max.y, input_trimesh.get_aabb().max.y);
    ASSERT_SCALAR_EQ(trimesh.get_aabb().max.z, input_trimesh.get_aabb().max.z);

    // Check vertices, indices, edge angles...
    for (size_t i = 0; i < trimesh.m_vertices.size(); ++i) {
        ASSERT_SCALAR_EQ(trimesh.m_vertices[i].x, input_trimesh.m_vertices[i].x);
        ASSERT_SCALAR_EQ(trimesh.m_vertices[i].y, input_trimesh.m_vertices[i].y);
        ASSERT_SCALAR_EQ(trimesh.m_vertices[i].z, input_trimesh.m_vertices[i].z);
    }

    for (size_t i = 0; i < trimesh.m_indices.size(); ++i) {
        ASSERT_EQ(trimesh.m_indices[i], input_trimesh.m_indices[i]);
    }

    for (size_t i = 0; i < trimesh.m_is_concave_edge.size(); ++i) {
        ASSERT_EQ(trimesh.m_is_concave_edge[i], input_trimesh.m_is_concave_edge[i]);
    }

    // Check trees.
    for (size_t i = 0; i < trimesh.m_triangle_tree.m_nodes.size(); ++i) {
        ASSERT_SCALAR_EQ(trimesh.m_triangle_tree.m_nodes[i].aabb.min.x, input_trimesh.m_triangle_tree.m_nodes[i].aabb.min.x);
        ASSERT_SCALAR_EQ(trimesh.m_triangle_tree.m_nodes[i].aabb.min.y, input_trimesh.m_triangle_tree.m_nodes[i].aabb.min.y);
        ASSERT_SCALAR_EQ(trimesh.m_triangle_tree.m_nodes[i].aabb.min.z, input_trimesh.m_triangle_tree.m_nodes[i].aabb.min.z);

        ASSERT_SCALAR_EQ(trimesh.m_triangle_tree.m_nodes[i].aabb.max.x, input_trimesh.m_triangle_tree.m_nodes[i].aabb.max.x);
        ASSERT_SCALAR_EQ(trimesh.m_triangle_tree.m_nodes[i].aabb.max.y, input_trimesh.m_triangle_tree.m_nodes[i].aabb.max.y);
        ASSERT_SCALAR_EQ(trimesh.m_triangle_tree.m_nodes[i].aabb.max.z, input_trimesh.m_triangle_tree.m_nodes[i].aabb.max.z);

        ASSERT_EQ(trimesh.m_triangle_tree.m_nodes[i].child1, input_trimesh.m_triangle_tree.m_nodes[i].child1);
        ASSERT_EQ(trimesh.m_triangle_tree.m_nodes[i].child2, input_trimesh.m_triangle_tree.m_nodes[i].child2);
        ASSERT_EQ(trimesh.m_triangle_tree.m_nodes[i].id, input_trimesh.m_triangle_tree.m_nodes[i].id);
    }
}
