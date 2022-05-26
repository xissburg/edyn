#include "edyn/shapes/cylinder_shape.hpp"
#include "edyn/util/shape_util.hpp"

namespace edyn {

scalar cylinder_shape::support_projection(const vector3 &pos, const quaternion &orn, const vector3 &dir) const {
    return cylinder_support_projection(radius, half_length, axis, pos, orn, dir);
}

vector3 cylinder_shape::support_point(const vector3 &pos, const quaternion &orn, const vector3 &dir) const {
    return cylinder_support_point(radius, half_length, axis, pos, orn, dir);
}

void cylinder_shape::support_feature(const vector3 &dir, cylinder_feature &out_feature,
                                     size_t &out_feature_index, scalar threshold) const {
    // The feature is a cap face if the projection of a cap face onto dir is
    // smaller than threshold. The distance between the tip of dir (which is
    // unit length) and the cylinder axis is the sine of the angle between dir and
    // the cylinder axis. The projection is the diameter times the sine of this angle.

    // Index of vector element in cylinder object space that represents the
    // cylinder axis.
    auto cyl_ax_idx = static_cast<std::underlying_type_t<coordinate_axis>>(axis);
    // Index of vector elements orthogonal to cylinder axis.
    auto cyl_ax_ortho_idx0 = (cyl_ax_idx + 1) % 3;
    auto cyl_ax_ortho_idx1 = (cyl_ax_idx + 2) % 3;

    auto ortho_dir_len_sqr = dir[cyl_ax_ortho_idx0] * dir[cyl_ax_ortho_idx0] + dir[cyl_ax_ortho_idx1] * dir[cyl_ax_ortho_idx1];
    auto proj_cap_face_sqr = scalar(4) * radius * radius * ortho_dir_len_sqr;

    if (proj_cap_face_sqr < threshold * threshold) {
        out_feature = cylinder_feature::face;
        out_feature_index = dir[cyl_ax_idx] > 0 ? 0 : 1;
        return;
    }

    // The feature is the side edge if the projection of the cylinder axis onto
    // dir is smaller than threshold. `dir[cyl_ax_idx]` is the sine of the angle
    // between dir and the plane orthogonal to the cylinder. The projection is
    // the length times the sine.
    auto proj_side_edge = std::abs(scalar(2) * half_length * dir[cyl_ax_idx]);

    if (proj_side_edge < threshold) {
        out_feature = cylinder_feature::side_edge;
        return;
    }

    out_feature = cylinder_feature::cap_edge;
    out_feature_index = dir[cyl_ax_idx] > 0 ? 0 : 1;
}

void cylinder_shape::support_feature(const vector3 &pos, const quaternion &orn, const vector3 &axis_dir,
                                     cylinder_feature &out_feature, size_t &out_feature_index,
                                     scalar threshold) const {
    auto local_dir = rotate(conjugate(orn), axis_dir);
    support_feature(local_dir, out_feature, out_feature_index, threshold);
}

}
