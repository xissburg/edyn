#ifndef EDYN_SHAPES_ROTATED_MESH_HPP
#define EDYN_SHAPES_ROTATED_MESH_HPP

#include <vector>
#include "edyn/math/vector3.hpp"

namespace edyn {

/**
 * @brief Accompanying component for `covex_mesh`es containg their 
 * rotated vertices and normals to prevent repeated recalculation of
 * these values.
 */
struct rotated_mesh {
    std::vector<vector3> vertices;
    std::vector<vector3> normals;
};

}

#endif // EDYN_SHAPES_ROTATED_MESH_HPP
