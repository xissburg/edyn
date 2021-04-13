#ifndef EDYN_SHAPES_CAPSULE_SHAPE_HPP
#define EDYN_SHAPES_CAPSULE_SHAPE_HPP

#include "edyn/comp/aabb.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/quaternion.hpp"

namespace edyn {

struct capsule_shape {
    scalar radius;
    scalar half_length;

    std::array<vector3, 2> get_vertices(const vector3 &pos, const quaternion &orn) const {
        const auto capsule_axis = quaternion_x(orn);
        return {
            pos - capsule_axis * half_length,
            pos + capsule_axis * half_length
        };
    }
};

}

#endif // EDYN_SHAPES_CAPSULE_SHAPE_HPP