#include "edyn/shapes/cylinder_shape.hpp"

namespace edyn {

void cylinder_shape::support_feature(const vector3 &dir, cylinder_feature &out_feature, 
                         size_t &out_feature_index, vector3 &out_support_point, 
                         scalar &out_projection, scalar threshold) const {
    /* auto max_face_angle = threshold / (2 * radius);
    auto face_angle_sqr = (dir.y * dir.y + dir.z * dir.z) / (half_length * half_length);

    if (face_angle_sqr < max_face_angle * max_face_angle) {
        out_feature = CYLINDER_FEATURE_FACE;
        out_feature_index = dir.x > 0 ? 0 : 1;
        out_projection = 
        return;
    } 

    auto max_edge_angle = threshold / (2 * half_length);
    auto edge_angle_sqr = dir.x * dir.x / (dir.y * dir.y + dir.z * dir.z);

    if (edge_angle_sqr < max_edge_angle * max_edge_angle) {
        out_feature = CYLINDER_FEATURE_SIDE_EDGE;
        out_projection = radius;
        return;
    } */

    if (std::abs(dir.x) + EDYN_EPSILON >= scalar(1)) {
        out_feature = CYLINDER_FEATURE_FACE;
        out_feature_index = dir.x > 0 ? 0 : 1;
        out_projection = half_length;
    } else if (std::abs(dir.x) < EDYN_EPSILON) {
        out_feature = CYLINDER_FEATURE_SIDE_EDGE;
        out_projection = radius;
    } else {
        out_feature = CYLINDER_FEATURE_FACE_EDGE;

        auto len_yz_sqr = dir.y * dir.y + dir.z * dir.z;
        EDYN_ASSERT(len_yz_sqr > EDYN_EPSILON);
        auto len_yz_inv = radius / std::sqrt(len_yz_sqr);
        out_support_point.x = half_length * (dir.x > 0 ? 1 : -1);
        out_support_point.y = dir.y * len_yz_inv;
        out_support_point.z = dir.z * len_yz_inv;
        
        out_feature_index = dir.x > 0 ? 0 : 1;
        out_projection = dot(dir, out_support_point);
    }
}

void cylinder_shape::support_feature(const vector3 &pos, const quaternion &orn, 
                        const vector3 &axis_pos, const vector3 &axis_dir,
                        cylinder_feature &out_feature, size_t &out_feature_index,
                        vector3 &out_support_point, scalar &out_projection,
                        scalar threshold) const {
    auto local_dir = rotate(conjugate(orn), axis_dir);
    support_feature(local_dir, out_feature, out_feature_index, out_support_point, out_projection, threshold);
    out_support_point = pos + rotate(orn, out_support_point);
    out_projection += dot(pos - axis_pos, axis_dir);
}

}