#include "edyn/util/aabb_util.hpp"
#include "edyn/util/shape_util.hpp"
#include "edyn/math/coordinate_axis.hpp"
#include "edyn/math/transform.hpp"
#include <variant>

namespace edyn {

static constexpr scalar aabb_half_extent = 99999;

AABB plane_aabb(const vector3 &normal, scalar constant) {
    vector3 unit_min, unit_max;

    if (normal == vector3_x) {
        unit_min = {-1, -1, -1};
        unit_max = { 0,  1,  1};
    } else if (normal == -vector3_x) {
        unit_min = { 0, -1, -1};
        unit_max = { 1,  1,  1};
    } else if (normal == vector3_y) {
        unit_min = {-1, -1, -1};
        unit_max = { 1,  0,  1};
    } else if (normal == -vector3_y) {
        unit_min = {-1,  0, -1};
        unit_max = { 1,  1,  1};
    } else if (normal == vector3_z) {
        unit_min = {-1, -1, -1};
        unit_max = { 1,  1,  0};
    } else if (normal == -vector3_z) {
        unit_min = {-1, -1,  0};
        unit_max = { 1,  1,  1};
    } else {
        unit_min = {-1, -1, -1};
        unit_max = { 1,  1,  1};
    }

    auto pos_world = normal * constant;
    return {unit_min * aabb_half_extent + pos_world,
            unit_max * aabb_half_extent + pos_world};
}

AABB box_aabb(const vector3 &half_extents, const vector3 &pos, const quaternion &orn) {
    // Reference: Real-Time Collision Detection - Christer Ericson, section 4.2.6.
    auto aabb = AABB{pos, pos};
    auto basis = to_matrix3x3(orn);

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            auto e = basis[i][j] * -half_extents[j];
            auto f = -e;

            if (e < f) {
                aabb.min[i] += e;
                aabb.max[i] += f;
            } else {
                aabb.min[i] += f;
                aabb.max[i] += e;
            }
        }
    }

    return aabb;
}

AABB sphere_aabb(scalar radius, const vector3 &pos) {
    return {
        {pos.x - radius, pos.y - radius, pos.z - radius},
        {pos.x + radius, pos.y + radius, pos.z + radius}
    };
}

AABB cylinder_aabb(scalar radius, scalar half_length, coordinate_axis axis, const vector3 &pos, const quaternion &orn) {
    auto ptx = cylinder_support_point(radius, half_length, axis, orn, vector3_x);
    auto pty = cylinder_support_point(radius, half_length, axis, orn, vector3_y);
    auto ptz = cylinder_support_point(radius, half_length, axis, orn, vector3_z);
    auto v = vector3 {ptx.x, pty.y, ptz.z};

    return {pos - v, pos + v};
}

AABB capsule_aabb(scalar radius, scalar half_length, coordinate_axis axis, const vector3 &pos, const quaternion &orn) {
    auto dir = coordinate_axis_vector(axis, orn);
    auto v = dir * half_length;
    auto p0 = pos - v;
    auto p1 = pos + v;
    auto offset = vector3 {radius, radius, radius};
    return {min(p0, p1) - offset, max(p0, p1) + offset};
}

AABB aabb_to_world_space(const AABB &aabb, const vector3 &pos, const quaternion &orn) {
    auto center_local = (aabb.min + aabb.max) * scalar(0.5);
    auto center = to_world_space(center_local, pos, orn);
    auto extents = aabb.max - aabb.min;
    auto half_extents = extents * scalar(0.5);

    // Compute support points of the global axes.
    auto c_orn = conjugate(orn);
    auto axis_x = rotate(c_orn, vector3_x);
    auto axis_y = rotate(c_orn, vector3_y);
    auto axis_z = rotate(c_orn, vector3_z);

    auto sup_x = support_point_box(half_extents, axis_x);
    auto sup_y = support_point_box(half_extents, axis_y);
    auto sup_z = support_point_box(half_extents, axis_z);

    auto result_half_extents = vector3{
        dot(sup_x, axis_x),
        dot(sup_y, axis_y),
        dot(sup_z, axis_z)
    };
    auto result_min = center - result_half_extents;
    auto result_max = center + result_half_extents;
    return {result_min, result_max};
}

AABB aabb_to_object_space(const AABB &aabb, const vector3 &pos, const quaternion &orn) {
    auto center_local = (aabb.min + aabb.max) * scalar(0.5);
    auto center = to_object_space(center_local, pos, orn);
    auto extents = aabb.max - aabb.min;
    auto half_extents = extents * scalar(0.5);

    // Compute support points on local axes.
    auto axis_x = rotate(orn, vector3_x);
    auto axis_y = rotate(orn, vector3_y);
    auto axis_z = rotate(orn, vector3_z);

    auto sup_x = support_point_box(half_extents, axis_x);
    auto sup_y = support_point_box(half_extents, axis_y);
    auto sup_z = support_point_box(half_extents, axis_z);

    auto result_half_extents = vector3{
        dot(sup_x, axis_x),
        dot(sup_y, axis_y),
        dot(sup_z, axis_z)
    };
    auto result_min = center - result_half_extents;
    auto result_max = center + result_half_extents;
    return {result_min, result_max};
}

AABB point_cloud_aabb(const std::vector<vector3> &points) {
    // TODO: implement and use `parallel_reduce`.
    auto aabb = AABB{vector3_max, -vector3_max};

    for (auto &point : points) {
        aabb.min = min(aabb.min, point);
        aabb.max = max(aabb.max, point);
    }

    return aabb;
}

AABB point_cloud_aabb(const std::vector<vector3> &points,
                      const vector3 &pos, const quaternion &orn) {
    // TODO: implement and use `parallel_reduce`.
    auto aabb = AABB{vector3_max, -vector3_max};

    for (auto &point_local : points) {
        auto point_world = to_world_space(point_local, pos, orn);
        aabb.min = min(aabb.min, point_world);
        aabb.max = max(aabb.max, point_world);
    }

    return aabb;
}

AABB shape_aabb(const plane_shape &sh, const vector3 &pos, const quaternion &orn) {
    // Position and orientation are ignored for planes.
    return plane_aabb(sh.normal, sh.constant);
}

AABB shape_aabb(const sphere_shape &sh, const vector3 &pos, const quaternion &orn) {
    return sphere_aabb(sh.radius, pos);
}

AABB shape_aabb(const cylinder_shape &sh, const vector3 &pos, const quaternion &orn) {
    return cylinder_aabb(sh.radius, sh.half_length, sh.axis, pos, orn);
}

AABB shape_aabb(const capsule_shape &sh, const vector3 &pos, const quaternion &orn) {
    return capsule_aabb(sh.radius, sh.half_length, sh.axis, pos, orn);
}

AABB shape_aabb(const mesh_shape &sh, const vector3 &pos, const quaternion &orn) {
    return {
        sh.trimesh->get_aabb().min + pos,
        sh.trimesh->get_aabb().max + pos
    };
}

AABB shape_aabb(const box_shape &sh, const vector3 &pos, const quaternion &orn) {
    return box_aabb(sh.half_extents, pos, orn);
}

AABB shape_aabb(const polyhedron_shape &sh, const vector3 &pos, const quaternion &orn) {
    return point_cloud_aabb(sh.mesh->vertices, pos, orn);
}

AABB shape_aabb(const paged_mesh_shape &sh, const vector3 &pos, const quaternion &orn) {
    return {
        sh.trimesh->get_aabb().min + pos,
        sh.trimesh->get_aabb().max + pos
    };
}

AABB shape_aabb(const compound_shape &sh, const vector3 &pos, const quaternion &orn) {
    // Using AABB of transformed AABB for greater performance.
    auto aabb = aabb_to_world_space(sh.nodes.front().aabb, pos, orn);

    for (size_t i = 1; i < sh.nodes.size(); ++i) {
        auto aabb_i = aabb_to_world_space(sh.nodes[i].aabb, pos, orn);
        aabb = enclosing_aabb(aabb, aabb_i);
    }

    return aabb;
}

AABB shape_aabb(const shapes_variant_t &var, const vector3 &pos, const quaternion &orn) {
    AABB aabb;
    std::visit([&](auto &&shape) {
        aabb = shape_aabb(shape, pos, orn);
    }, var);
    return aabb;
}

}
