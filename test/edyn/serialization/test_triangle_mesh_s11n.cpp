#include "../common/common.hpp"
#include "edyn/util/shape_util.hpp"

TEST(triangle_mesh_serialization, test) {
    // Create triangle mesh.
    auto extent_x = 2;
    auto extent_z = 12;
    auto num_vertices_x = 4;
    auto num_vertices_z = 24;
    std::vector<edyn::vector3> vertices;
    std::vector<edyn::triangle_mesh::index_type> indices;
    edyn::make_plane_mesh(extent_x, extent_z, num_vertices_x, num_vertices_z,
                          vertices, indices);

    for (int z = 0; z < num_vertices_z; ++z) {
        auto t = (edyn::scalar(z) / edyn::scalar(num_vertices_z - 1)) * 2 - 1;
        auto y = (t*t*t*t - t*t) * 1.2;

        for (int x = 0; x < num_vertices_x; ++x) {
            vertices[z * num_vertices_x + x].y = y + (x == 0 || x == num_vertices_x - 1 ? 0.1 : 0);
        }
    }

    auto trimesh = edyn::triangle_mesh();
    trimesh.insert_vertices(vertices.begin(), vertices.end());
    trimesh.insert_indices(indices.begin(), indices.end());
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
    ASSERT_EQ(trimesh.num_vertices(), input_trimesh.num_vertices());
    ASSERT_EQ(trimesh.num_triangles(), input_trimesh.num_triangles());
    ASSERT_EQ(trimesh.num_edges(), input_trimesh.num_edges());

    // Check AABBs.
    ASSERT_SCALAR_EQ(trimesh.get_aabb().min.y, input_trimesh.get_aabb().min.y);
    ASSERT_SCALAR_EQ(trimesh.get_aabb().min.x, input_trimesh.get_aabb().min.x);
    ASSERT_SCALAR_EQ(trimesh.get_aabb().min.z, input_trimesh.get_aabb().min.z);
    ASSERT_SCALAR_EQ(trimesh.get_aabb().max.x, input_trimesh.get_aabb().max.x);
    ASSERT_SCALAR_EQ(trimesh.get_aabb().max.y, input_trimesh.get_aabb().max.y);
    ASSERT_SCALAR_EQ(trimesh.get_aabb().max.z, input_trimesh.get_aabb().max.z);

    // Check vertices, indices, edge angles...
    for (size_t i = 0; i < trimesh.num_vertices(); ++i) {
        ASSERT_SCALAR_EQ(trimesh.get_vertex_position(i).x, input_trimesh.get_vertex_position(i).x);
        ASSERT_SCALAR_EQ(trimesh.get_vertex_position(i).y, input_trimesh.get_vertex_position(i).y);
        ASSERT_SCALAR_EQ(trimesh.get_vertex_position(i).z, input_trimesh.get_vertex_position(i).z);
    }

    for (size_t i = 0; i < trimesh.num_triangles(); ++i) {
        for (size_t j = 0; j < 3; ++j) {
            ASSERT_EQ(trimesh.get_face_vertex_index(i, j), input_trimesh.get_face_vertex_index(i, j));
        }
    }

    for (size_t i = 0; i < trimesh.num_edges(); ++i) {
        ASSERT_EQ(trimesh.is_convex_edge(i), input_trimesh.is_convex_edge(i));
    }
}
