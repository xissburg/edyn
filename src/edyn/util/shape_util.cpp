#include "edyn/util/shape_util.hpp"

namespace edyn {

void make_plane_mesh(scalar extent_x, scalar extent_z, 
                     size_t num_vertices_x, size_t num_vertices_z, 
                     std::vector<vector3> &vertices, std::vector<uint16_t> &indices) {
    const auto half_extent_x = extent_x * scalar(0.5);
    const auto half_extent_z = extent_z * scalar(0.5);
    const auto quad_size_x = extent_x / (num_vertices_x - 1);
    const auto quad_size_z = extent_z / (num_vertices_z - 1);

    for (size_t j = 0; j < num_vertices_z; ++j) {
        auto z = -half_extent_z + j * quad_size_z;
        for (size_t i = 0; i < num_vertices_x; ++i) {
            vertices.push_back({-half_extent_x + i * quad_size_x, 0, z});
        }
    }

    for (size_t j = 0; j < num_vertices_z - 1; ++j) {
        for (size_t i = 0; i < num_vertices_x - 1; ++i) {
            indices.push_back(j * num_vertices_x + i + 1);
            indices.push_back(j * num_vertices_x + i);
            indices.push_back((j + 1) * num_vertices_x + i);

            indices.push_back((j + 1) * num_vertices_x + i);
            indices.push_back((j + 1) * num_vertices_x + i + 1);
            indices.push_back(j * num_vertices_x + i + 1);
        }
    }
}

}