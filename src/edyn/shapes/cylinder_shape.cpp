#include "edyn/shapes/cylinder_shape.hpp"

namespace edyn {

AABB cylinder_shape::aabb(const vector3 &pos, const quaternion &orn) const {
    auto ptx = support_point(orn, vector3_x);
    auto pty = support_point(orn, vector3_y);
    auto ptz = support_point(orn, vector3_z);
    auto v = vector3 {ptx.x, pty.y, ptz.z};

    return {pos - v, pos + v};
}

vector3 cylinder_shape::support_point(const vector3 &dir) const {
    // Squared length in yz plane.
    auto lyz2 = dir.y * dir.y + dir.z * dir.z;

    if (lyz2 > EDYN_EPSILON) {
        auto d = radius / std::sqrt(lyz2);
        return {dir.x < 0 ? -half_length : half_length, dir.y * d, dir.z * d};
    } 
    
    return {dir.x < 0 ? -half_length : half_length, radius, 0};
}

vector3 cylinder_shape::support_point(const quaternion &orn, const vector3 &dir) const {
    auto local_dir = rotate(conjugate(orn), dir);
    auto pt = support_point(local_dir);
    return rotate(orn, pt);
}

vector3 cylinder_shape::support_point(const vector3 &pos, const quaternion &orn, const vector3 &dir) const {
    return pos + support_point(orn, dir);
}

scalar cylinder_shape::support_projection(const vector3 &pos, const quaternion &orn, const vector3 &dir) const {
    auto local_dir = rotate(conjugate(orn), dir);
    auto pt = support_point(local_dir);
    return dot(pos, dir) + dot(pt, local_dir);
}

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