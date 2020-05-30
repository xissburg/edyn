#include "../common/common.hpp"

TEST(triangle_mesh_serialization, test) {
    // Create triangle mesh.
    auto trimesh = edyn::triangle_mesh();

    auto extent_x = 2;
    auto extent_z = 12;
    auto num_vertices_x = 4;
    auto num_vertices_z = 24;
    edyn::make_plane_mesh(extent_x, extent_z, num_vertices_x, num_vertices_z, 
                          trimesh.vertices, trimesh.indices);

    for (int z = 0; z < num_vertices_z; ++z) {
        auto t = (edyn::scalar(z) / edyn::scalar(num_vertices_z - 1)) * 2 - 1;
        auto y = (t*t*t*t - t*t) * 1.2;
        
        for (int x = 0; x < num_vertices_x; ++x) {
            trimesh.vertices[z * num_vertices_x + x].y = y + (x == 0 || x == num_vertices_x - 1 ? 0.1 : 0);
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
    ASSERT_EQ(trimesh.vertices.size()       , input_trimesh.vertices.size());
    ASSERT_EQ(trimesh.indices.size()        , input_trimesh.indices.size());
    ASSERT_EQ(trimesh.cos_angles.size()     , input_trimesh.cos_angles.size());
    ASSERT_EQ(trimesh.is_concave_edge.size(), input_trimesh.is_concave_edge.size());
    ASSERT_EQ(trimesh.tree.m_nodes.size()   , input_trimesh.tree.m_nodes.size());

    // Check AABBs.
    ASSERT_SCALAR_EQ(trimesh.aabb.min.x, input_trimesh.aabb.min.x);
    ASSERT_SCALAR_EQ(trimesh.aabb.min.y, input_trimesh.aabb.min.y);
    ASSERT_SCALAR_EQ(trimesh.aabb.min.z, input_trimesh.aabb.min.z);
    ASSERT_SCALAR_EQ(trimesh.aabb.max.x, input_trimesh.aabb.max.x);
    ASSERT_SCALAR_EQ(trimesh.aabb.max.y, input_trimesh.aabb.max.y);
    ASSERT_SCALAR_EQ(trimesh.aabb.max.z, input_trimesh.aabb.max.z);

    // Check vertices, indices, edge angles...
    for (size_t i = 0; i < trimesh.vertices.size(); ++i) {
        ASSERT_SCALAR_EQ(trimesh.vertices[i].x, input_trimesh.vertices[i].x);
        ASSERT_SCALAR_EQ(trimesh.vertices[i].y, input_trimesh.vertices[i].y);
        ASSERT_SCALAR_EQ(trimesh.vertices[i].z, input_trimesh.vertices[i].z);
    }

    for (size_t i = 0; i < trimesh.indices.size(); ++i) {
        ASSERT_EQ(trimesh.indices[i], input_trimesh.indices[i]);
    }

    for (size_t i = 0; i < trimesh.cos_angles.size(); ++i) {
        ASSERT_SCALAR_EQ(trimesh.cos_angles[i], input_trimesh.cos_angles[i]);
    }

    for (size_t i = 0; i < trimesh.is_concave_edge.size(); ++i) {
        ASSERT_EQ(trimesh.is_concave_edge[i], input_trimesh.is_concave_edge[i]);
    }

    // Check trees.
    for (size_t i = 0; i < trimesh.tree.m_nodes.size(); ++i) {
        ASSERT_SCALAR_EQ(trimesh.tree.m_nodes[i].aabb.min.x, input_trimesh.tree.m_nodes[i].aabb.min.x);
        ASSERT_SCALAR_EQ(trimesh.tree.m_nodes[i].aabb.min.y, input_trimesh.tree.m_nodes[i].aabb.min.y);
        ASSERT_SCALAR_EQ(trimesh.tree.m_nodes[i].aabb.min.z, input_trimesh.tree.m_nodes[i].aabb.min.z);

        ASSERT_SCALAR_EQ(trimesh.tree.m_nodes[i].aabb.max.x, input_trimesh.tree.m_nodes[i].aabb.max.x);
        ASSERT_SCALAR_EQ(trimesh.tree.m_nodes[i].aabb.max.y, input_trimesh.tree.m_nodes[i].aabb.max.y);
        ASSERT_SCALAR_EQ(trimesh.tree.m_nodes[i].aabb.max.z, input_trimesh.tree.m_nodes[i].aabb.max.z);

        ASSERT_EQ(trimesh.tree.m_nodes[i].child1, input_trimesh.tree.m_nodes[i].child1);
        ASSERT_EQ(trimesh.tree.m_nodes[i].child2, input_trimesh.tree.m_nodes[i].child2);
        ASSERT_EQ(trimesh.tree.m_nodes[i].id, input_trimesh.tree.m_nodes[i].id);
    }
}