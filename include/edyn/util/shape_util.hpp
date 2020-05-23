#ifndef EDYN_UTIL_SHAPE_UTIL_HPP
#define EDYN_UTIL_SHAPE_UTIL_HPP

#include "edyn/math/vector3.hpp"
#include <vector>
#include <cstdint>
#include <string>

namespace edyn {

void make_plane_mesh(scalar extent_x, scalar extent_z, 
                     size_t num_vertices_x, size_t num_vertices_z, 
                     std::vector<vector3> &vertices, std::vector<uint16_t> &indices);

bool load_mesh_from_obj(const std::string &path, 
                        std::vector<vector3> &vertices, 
                        std::vector<uint16_t> &indices);

}

#endif // EDYN_UTIL_SHAPE_UTIL_HPP