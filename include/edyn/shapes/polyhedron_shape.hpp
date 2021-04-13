#ifndef EDYN_SHAPES_POLYHEDRON_HPP
#define EDYN_SHAPES_POLYHEDRON_HPP

#include <memory>
#include "edyn/shapes/convex_mesh.hpp"
#include "edyn/comp/aabb.hpp"
#include "edyn/math/matrix3x3.hpp"

namespace edyn {

struct polyhedron_shape {
    std::shared_ptr<convex_mesh> mesh;
    AABB local_aabb;

    polyhedron_shape() = default;
    polyhedron_shape(const std::string &path_to_obj);

    void calculate_local_aabb();

    AABB aabb(const vector3 &pos, const quaternion &orn) const;
};

}

#endif // EDYN_SHAPES_POLYHEDRON_HPP
