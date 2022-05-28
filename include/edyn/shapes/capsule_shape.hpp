#ifndef EDYN_SHAPES_CAPSULE_SHAPE_HPP
#define EDYN_SHAPES_CAPSULE_SHAPE_HPP

#include <array>
#include <cstdint>
#include "edyn/math/quaternion.hpp"
#include "edyn/math/coordinate_axis.hpp"

namespace edyn {

enum class capsule_feature : uint8_t {
    hemisphere,
    side
};

struct capsule_shape {
    scalar radius;
    scalar half_length;
    coordinate_axis axis {coordinate_axis::x};

    auto get_vertices(const vector3 &pos, const quaternion &orn) const {
        const auto dir = coordinate_axis_vector(axis, orn);
        return std::array<vector3, 2>{
            pos + dir * half_length,
            pos - dir * half_length
        };
    }
};

template<typename Archive>
void serialize(Archive &archive, capsule_shape &s) {
    archive(s.half_length);
    archive(s.radius);
    archive(s.axis);
}

}

#endif // EDYN_SHAPES_CAPSULE_SHAPE_HPP
