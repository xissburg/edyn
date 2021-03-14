#include "edyn/shapes/cylinder_shape.hpp"

namespace edyn {

void cylinder_shape::support_feature(const vector3 &dir, cylinder_feature &out_feature, 
                         size_t &out_feature_index, vector3 &out_support_point, 
                         scalar &out_projection, scalar threshold) const {
    out_support_point = support_point(dir);
    out_projection = dot(out_support_point, dir);

    auto max_face_angle = threshold / (2 * radius);
    auto face_angle_sqr = (dir.y * dir.y + dir.z * dir.z) / (half_length * half_length);

    if (face_angle_sqr < max_face_angle * max_face_angle) {
        out_feature = cylinder_feature::face;
        out_feature_index = dir.x > 0 ? 0 : 1;
        return;
    } 

    auto max_edge_angle = threshold / (2 * half_length);
    auto edge_angle_sqr = dir.x * dir.x / (dir.y * dir.y + dir.z * dir.z);

    if (edge_angle_sqr < max_edge_angle * max_edge_angle) {
        out_feature = cylinder_feature::side_edge;
        return;
    }

    out_feature = cylinder_feature::cap_edge;
    out_feature_index = dir.x > 0 ? 0 : 1;

    /* if (std::abs(dir.x) + EDYN_EPSILON >= scalar(1)) {
        out_feature = cylinder_feature::face;
        out_feature_index = dir.x > 0 ? 0 : 1;
        out_projection = half_length;
    } else if (std::abs(dir.x) < EDYN_EPSILON) {
        out_feature = cylinder_feature::side_edge;
        out_projection = radius;
    } else {
        out_feature = cylinder_feature::cap_edge;

        auto len_yz_sqr = dir.y * dir.y + dir.z * dir.z;
        out_support_point.x = half_length * (dir.x > 0 ? 1 : -1);

        if (len_yz_sqr > EDYN_EPSILON) {
            auto r_len_yz_inv = radius / std::sqrt(len_yz_sqr);
            out_support_point.y = dir.y * r_len_yz_inv;
            out_support_point.z = dir.z * r_len_yz_inv;
        } else {
            out_support_point.y = 0;
            out_support_point.z = radius;
        }
        
        out_feature_index = dir.x > 0 ? 0 : 1;
        out_projection = dot(dir, out_support_point);
    } */
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