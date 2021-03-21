#ifndef EDYN_SHAPES_CYLINDER_SHAPE_HPP
#define EDYN_SHAPES_CYLINDER_SHAPE_HPP

#include "edyn/comp/aabb.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/util/moment_of_inertia.hpp"

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

    AABB aabb(const vector3 &pos, const quaternion &orn) const {
        auto ptx = support_point(orn, vector3_x);
        auto pty = support_point(orn, vector3_y);
        auto ptz = support_point(orn, vector3_z);
        auto v = vector3 {ptx.x, pty.y, ptz.z};

        return {pos - v, pos + v};
    }

    matrix3x3 inertia(scalar mass) const {
        return diagonal_matrix(moment_of_inertia_solid_cylinder(mass, half_length * 2, radius));
    }

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

    void support_feature(const vector3 &dir, cylinder_feature &out_feature, 
                         size_t &out_feature_index, vector3 &out_support_point, 
                         scalar &out_projection, scalar threshold) const;

    void support_feature(const vector3 &pos, const quaternion &orn, 
                         const vector3 &axis_pos, const vector3 &axis_dir,
                         cylinder_feature &out_feature, size_t &out_feature_index,
                         vector3 &out_support_point, scalar &out_projection,
                         scalar threshold) const;
};

}

#endif // EDYN_SHAPES_CYLINDER_SHAPE_HPP