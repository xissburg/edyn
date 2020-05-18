#include "edyn/util/shape_util.hpp"
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

}