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

    // First element in `xyz` is the index of the cylinder axis, the other two
    // correspond to the axes in the plane orthogonal to the cylinder.
    int xyz[3];

    switch (axis) {
    case shape_axis::x:
        xyz[0] = 0; xyz[1] = 1; xyz[2] = 2;
        break;
    case shape_axis::y:
        xyz[0] = 1; xyz[1] = 2; xyz[2] = 0;
        break;
    case shape_axis::z:
        xyz[0] = 2; xyz[1] = 0; xyz[2] = 1;
        break;
    }

    auto proj_cap_face_sqr = scalar(4) * radius * radius * (dir[xyz[1]] * dir[xyz[1]] + dir[xyz[2]] * dir[xyz[2]]);

    if (proj_cap_face_sqr < threshold * threshold) {
        out_feature = cylinder_feature::face;
        out_feature_index = dir[xyz[0]] > 0 ? 0 : 1;
        return;
    }

    // The feature is the side edge if the projection of the cylinder axis onto
    // dir is smaller than threshold. `dir[xyz[0]]` is the sine of the angle
    // between dir and the plane orthogonal to the cylinder. The projection is
    // the length times the sine.
    auto proj_side_edge = std::abs(scalar(2) * half_length * dir[xyz[0]]);

    if (proj_side_edge < threshold) {
        out_feature = cylinder_feature::side_edge;
        return;
    }

    out_feature = cylinder_feature::cap_edge;
    out_feature_index = dir[xyz[0]] > 0 ? 0 : 1;
}

void cylinder_shape::support_feature(const vector3 &pos, const quaternion &orn, const vector3 &axis_dir,
                                     cylinder_feature &out_feature, size_t &out_feature_index,
                                     scalar threshold) const {
    auto local_dir = rotate(conjugate(orn), axis_dir);
    support_feature(local_dir, out_feature, out_feature_index, threshold);
}

}
