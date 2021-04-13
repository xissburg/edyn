#ifndef EDYN_SHAPES_CYLINDER_SHAPE_HPP
#define EDYN_SHAPES_CYLINDER_SHAPE_HPP

#include "edyn/comp/aabb.hpp"
#include "edyn/math/matrix3x3.hpp"
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

    AABB aabb(const vector3 &pos, const quaternion &orn) const;

    vector3 support_point(const vector3 &dir) const;

    vector3 support_point(const quaternion &orn, const vector3 &dir) const;
    
    vector3 support_point(const vector3 &pos, const quaternion &orn, 
                          const vector3 &dir) const;

    scalar support_projection(const vector3 &pos, const quaternion &orn, 
                              const vector3 &dir) const;

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