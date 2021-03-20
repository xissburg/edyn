#ifndef EDYN_SHAPES_POLYHEDRON_HPP
#define EDYN_SHAPES_POLYHEDRON_HPP

#include <vector>
#include <memory>
#include "edyn/comp/aabb.hpp"

namespace edyn {

struct convex_mesh {
    std::vector<vector3> vertices;
    std::vector<uint16_t> edges;
    std::vector<uint16_t> triangles;
};

struct polyhedron_shape {
    std::shared_ptr<convex_mesh> mesh;

    AABB aabb(const vector3 &pos, const quaternion &orn) const {
        return {};
    }

    vector3 inertia(scalar mass) const {
        return {};
    }
};

}

#endif // EDYN_SHAPES_POLYHEDRON_HPP
