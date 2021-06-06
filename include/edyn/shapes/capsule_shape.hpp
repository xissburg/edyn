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
