#ifndef EDYN_SHAPES_CAPSULE_SHAPE_HPP
#define EDYN_SHAPES_CAPSULE_SHAPE_HPP

#include <array>
#include <cstdint>
#include "edyn/math/quaternion.hpp"

namespace edyn {

enum class capsule_feature : uint8_t {
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

template<typename Archive>
void serialize(Archive &archive, capsule_shape &s) {
    archive(s.half_length);
    archive(s.radius);
}

}

#endif // EDYN_SHAPES_CAPSULE_SHAPE_HPP
