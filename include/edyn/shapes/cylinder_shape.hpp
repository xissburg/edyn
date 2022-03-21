#ifndef EDYN_SHAPES_CYLINDER_SHAPE_HPP
#define EDYN_SHAPES_CYLINDER_SHAPE_HPP

#include <array>
#include <cstdint>
#include "edyn/math/quaternion.hpp"

namespace edyn {

enum class cylinder_feature : uint8_t {
    // Either of the two cylinder caps.
    face,
    // An edge on the side wall of the cylinder.
    side_edge,
    // The edge/border of a cylinder cap.
    cap_edge,
};

struct cylinder_shape {
    scalar radius;
    scalar half_length;

    /**
     * @brief Get the world space position of the center of both cylinder
     * cap faces. The first is the center of the face along the positive axis
     * direction.
     * @param pos Position of center.
     * @param orn Cylinder orientation.
     * @return An array with two positions.
     */
    auto get_vertices(const vector3 &pos, const quaternion &orn) const {
        const auto axis = quaternion_x(orn);
        return std::array<vector3, 2>{
            pos + axis * half_length,
            pos - axis * half_length
        };
    }

    scalar support_projection(const vector3 &pos, const quaternion &orn,
                              const vector3 &dir) const;

    vector3 support_point(const vector3 &pos, const quaternion &orn,
                          const vector3 &dir) const;

    void support_feature(const vector3 &dir, cylinder_feature &out_feature,
                         size_t &out_feature_index, scalar threshold) const;

    void support_feature(const vector3 &pos, const quaternion &orn, const vector3 &axis_dir,
                         cylinder_feature &out_feature, size_t &out_feature_index,
                         scalar threshold) const;
};

template<typename Archive>
void serialize(Archive &archive, cylinder_shape &s) {
    archive(s.half_length);
    archive(s.radius);
}

}

#endif // EDYN_SHAPES_CYLINDER_SHAPE_HPP
