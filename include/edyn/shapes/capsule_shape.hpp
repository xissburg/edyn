#ifndef EDYN_SHAPES_CAPSULE_SHAPE_HPP
#define EDYN_SHAPES_CAPSULE_SHAPE_HPP

#include <array>
#include "edyn/math/quaternion.hpp"

namespace edyn {

enum class capsule_feature {
    hemisphere,
    side
};

struct capsule_shape {
    scalar radius;
    scalar half_length;

    auto get_vertices(const vector3 &pos, const quaternion &orn) const {
        const auto axis = quaternion_x(orn);
        return std::array<vector3, 2>{
            pos + axis * half_length,
            pos - axis * half_length
        };
    }
};

}

#endif // EDYN_SHAPES_CAPSULE_SHAPE_HPP
