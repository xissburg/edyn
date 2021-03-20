#ifndef EDYN_SHAPES_POLYHEDRON_HPP
#define EDYN_SHAPES_POLYHEDRON_HPP

#include <memory>
#include "edyn/shapes/convex_mesh.hpp"
#include "edyn/comp/aabb.hpp"

namespace edyn {

struct polyhedron_shape {
    std::shared_ptr<convex_mesh> mesh;
    AABB local_aabb;

    void calculate_local_aabb();

    AABB aabb(const vector3 &pos, const quaternion &orn) const;

    vector3 inertia(scalar mass) const;
};

}

#endif // EDYN_SHAPES_POLYHEDRON_HPP
