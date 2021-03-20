#ifndef EDYN_SHAPES_CONVEX_MESH_HPP
#define EDYN_SHAPES_CONVEX_MESH_HPP

#include <vector>
#include <cstdint>
#include "edyn/math/vector3.hpp"

namespace edyn {

struct convex_mesh {
    std::vector<vector3> vertices;
    std::vector<uint16_t> edges;
    std::vector<uint16_t> triangles;
};

}

#endif // EDYN_SHAPES_CONVEX_MESH_HPP
