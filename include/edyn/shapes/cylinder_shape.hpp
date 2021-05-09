#ifndef EDYN_SHAPES_CYLINDER_SHAPE_HPP
#define EDYN_SHAPES_CYLINDER_SHAPE_HPP

#include <array>
#include "edyn/math/quaternion.hpp"

namespace edyn {

enum class cylinder_feature {
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

    std::array<vector3, 2> get_vertices(const vector3 &pos, const quaternion &orn) const {
        const auto axis = quaternion_x(orn);
        return {
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

    void support_feature(const vector3 &pos, const quaternion &orn,
                         const vector3 &axis_pos, const vector3 &axis_dir,
                         cylinder_feature &out_feature, size_t &out_feature_index,
                         scalar threshold) const;
};

}

#endif // EDYN_SHAPES_CYLINDER_SHAPE_HPP