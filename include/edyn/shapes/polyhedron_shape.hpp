#ifndef EDYN_SHAPES_POLYHEDRON_HPP
#define EDYN_SHAPES_POLYHEDRON_HPP

#include <memory>
#include "edyn/shapes/convex_mesh.hpp"

namespace edyn {

struct polyhedron_shape {
    std::shared_ptr<convex_mesh> mesh;
    std::shared_ptr<rotated_mesh> rotated;

    polyhedron_shape() = default;
    polyhedron_shape(const std::string &path_to_obj,
                     const vector3 &pos = vector3_zero,
                     const quaternion &orn = quaternion_identity);
};

}

#endif // EDYN_SHAPES_POLYHEDRON_HPP
