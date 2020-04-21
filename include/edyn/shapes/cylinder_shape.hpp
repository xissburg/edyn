#ifndef EDYN_SHAPES_CYLINDER_SHAPE_HPP
#define EDYN_SHAPES_CYLINDER_SHAPE_HPP

#include "edyn/comp/aabb.hpp"
#include "edyn/math/quaternion.hpp"

namespace edyn {

enum cylinder_feature {
    // Either of the two cylinder caps.
    CYLINDER_FEATURE_FACE,
    // An edge on the side wall of the cylinder.
    CYLINDER_FEATURE_SIDE_EDGE,
    // The edge/border of a cylinder cap.
    CYLINDER_FEATURE_FACE_EDGE,
};

struct cylinder_shape {
    scalar radius;
    scalar half_length;

    vector3 support_point(const vector3 &dir) const {
        // Squared length in yz plane.
        auto lyz2 = dir.y * dir.y + dir.z * dir.z;

        if (lyz2 > EDYN_EPSILON) {
            auto d = radius / std::sqrt(lyz2);
            return {dir.x < 0 ? -half_length : half_length, dir.y * d, dir.z * d};
        } 
        
        return {dir.x < 0 ? -half_length : half_length, radius, 0};
    }

    vector3 support_point(const quaternion &orn, const vector3 &dir) const {
        auto local_dir = rotate(conjugate(orn), dir);
        auto pt = support_point(local_dir);
        return rotate(orn, pt);
    }
    
    vector3 support_point(const vector3 &pos, const quaternion &orn, const vector3 &dir) const {
        return pos + support_point(orn, dir);
    }

    AABB aabb(const vector3 &pos, const quaternion &orn) const {
        auto ptx = support_point(orn, vector3_x);
        auto pty = support_point(orn, vector3_y);
        auto ptz = support_point(orn, vector3_z);
        auto v = vector3 {ptx.x, pty.y, ptz.z};

        return {pos - v, pos + v};
    }

    vector3 inertia(scalar mass) const {
        auto len = half_length * 2;
        scalar xx = scalar(0.5) * mass * radius * radius;
        scalar yy_zz =  scalar(1) / scalar(12) * mass * (scalar(3) * radius * radius + len * len);
        return {xx, yy_zz, yy_zz};
    }
};

}

#endif // EDYN_SHAPES_CYLINDER_SHAPE_HPP