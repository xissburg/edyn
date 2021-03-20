#include "edyn/util/shape_util.hpp"
#include "edyn/math/quaternion.hpp"
#include <fstream>
#include <sstream>

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

bool load_mesh_from_obj(const std::string &path, 
                        std::vector<vector3> &vertices, 
                        std::vector<uint16_t> &indices) {
    auto file = std::ifstream(path);

    if (!file.is_open()) {
        return false;
    }

    std::string line;

    while (std::getline(file, line)) {
        auto pos_space = line.find(" ");

        if (pos_space == std::string::npos) {
            continue;
        }

        auto cmd = line.substr(0, pos_space);

        if (cmd == "v") {
            auto iss = std::istringstream(line.substr(pos_space, line.size() - pos_space));
            edyn::vector3 position;
            iss >> position.x >> position.y >> position.z;
            vertices.push_back(position);
        } else if (cmd == "f") {
            auto iss = std::istringstream(line.substr(pos_space, line.size() - pos_space));
            uint16_t idx[3];
            iss >> idx[0] >> idx[1] >> idx[2];
            indices.push_back(idx[0] - 1);
            indices.push_back(idx[1] - 1);
            indices.push_back(idx[2] - 1);
        }
    }

    return true;
}

vector3 support_point_box(const vector3 &half_extents, const vector3 &dir) {
    return {
        dir.x > 0 ? half_extents.x : -half_extents.x,
        dir.y > 0 ? half_extents.y : -half_extents.y,
        dir.z > 0 ? half_extents.z : -half_extents.z
    };
}

AABB aabb_of_aabb(const AABB &aabb, const vector3 &pos, const quaternion &orn) {
    auto center = (aabb.min + aabb.max) * scalar(0.5);
    auto center_world = to_world_space(center, pos, orn);
    auto extents = aabb.max - aabb.min;
    auto half_extents = extents * scalar(0.5);

    // Compute support points of the global axes.
    auto c_orn = conjugate(orn);
    auto axis_x = rotate(c_orn, vector3_x);
    auto axis_y = rotate(c_orn, vector3_y);
    auto axis_z = rotate(c_orn, vector3_z);

    auto sup_x = support_point_box(half_extents, axis_x);
    auto sup_y = support_point_box(half_extents, axis_y);
    auto sup_z = support_point_box(half_extents, axis_z);

    auto result_half_extent = vector3{
        dot(sup_x, axis_x),
        dot(sup_y, axis_y),
        dot(sup_z, axis_z)
    };
    auto result_min = center_world - result_half_extent;
    auto result_max = center_world + result_half_extent;
    return {result_min, result_max};
}

}