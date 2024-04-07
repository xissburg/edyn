#include "edyn/collision/collide.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/math.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/util/shape_util.hpp"
#include "edyn/math/triangle.hpp"
#include <cmath>

namespace edyn {

static void collide_sphere_triangle(
    const sphere_shape &sphere, const triangle_mesh &mesh, size_t tri_idx,
    const collision_context &ctx, collision_result &result) {

    const auto &sphere_pos = ctx.posA;
    const auto &sphere_orn = ctx.ornA;
    const auto tri_vertices = mesh.get_triangle_vertices(tri_idx);
    const auto tri_normal = mesh.get_triangle_normal(tri_idx);

    // Triangle normal.
    auto distance = dot(sphere_pos - tri_vertices[0], tri_normal) - sphere.radius;
    auto sep_axis = tri_normal;

    // Triangle edges.
    for (size_t i = 0; i < 3; ++i) {
        auto &v0 = tri_vertices[i];
        auto &v1 = tri_vertices[(i + 1) % 3];
        vector3 closest; scalar t;
        closest_point_segment(v0, v1, sphere_pos, t, closest);
        auto dir = sphere_pos - closest;

        if (!try_normalize(dir)) {
            continue;
        }

        auto projA = -(dot(sphere_pos, -dir) + sphere.radius);
        auto projB = get_triangle_support_projection(tri_vertices, dir);
        auto dist = projA - projB;

        if (dist > distance) {
            distance = dist;
            sep_axis = dir;
        }
    }

    if (distance > ctx.threshold) {
        return;
    }

    triangle_feature tri_feature;
    size_t tri_feature_index;
    scalar proj_tri;
    get_triangle_support_feature(tri_vertices, vector3_zero, sep_axis,
                                 tri_feature, tri_feature_index,
                                 proj_tri, support_feature_tolerance);

    sep_axis = clip_triangle_separating_axis(sep_axis, mesh, tri_idx, tri_vertices, tri_normal, tri_feature, tri_feature_index);

    get_triangle_support_feature(tri_vertices, vector3_zero, sep_axis,
                                 tri_feature, tri_feature_index,
                                 proj_tri, support_feature_tolerance);

    auto proj_sphere = -(dot(sphere_pos, -sep_axis) + sphere.radius);

    distance = proj_sphere - proj_tri;

    if (distance > ctx.threshold) {
        return;
    }

    if (-distance > mesh.get_thickness()) {
        return;
    }

    collision_result::collision_point point;
    point.normal = sep_axis;
    point.distance = distance;
    point.featureB = {tri_feature};
    point.featureB->index = get_triangle_mesh_feature_index(mesh, tri_idx, tri_feature, tri_feature_index);

    switch (tri_feature) {
    case triangle_feature::face: {
        if (point_in_triangle(tri_vertices, tri_normal, sphere_pos)) {
            point.pivotA = rotate(conjugate(sphere_orn), -tri_normal * sphere.radius);
            point.pivotB = project_plane(sphere_pos, tri_vertices[0], tri_normal);
            point.normal_attachment = contact_normal_attachment::normal_on_B;
            point.normal = tri_normal;
            result.maybe_add_point(point);
        }
        break;
    }
    case triangle_feature::edge: {
        auto &v0 = tri_vertices[tri_feature_index];
        auto &v1 = tri_vertices[(tri_feature_index + 1) % 3];

        vector3 pivotB; scalar t;
        closest_point_line(v0, v1 - v0, sphere_pos, t, pivotB);

        if (t > 0 && t < 1) {
            point.pivotA = rotate(conjugate(sphere_orn), -sep_axis * sphere.radius);
            point.pivotB = pivotB;
            point.normal_attachment = contact_normal_attachment::none;
            result.maybe_add_point(point);
        }
        break;
    }
    case triangle_feature::vertex: {
        point.pivotA = rotate(conjugate(sphere_orn), -sep_axis * sphere.radius);
        point.pivotB = tri_vertices[tri_feature_index];
        point.normal_attachment = contact_normal_attachment::none;
        result.maybe_add_point(point);
    }
    }
}

void collide(const sphere_shape &sphere, const triangle_mesh &mesh,
             const collision_context &ctx, collision_result &result) {
    const auto inset = vector3_one * -contact_breaking_threshold;
    const auto visit_aabb = ctx.aabbA.inset(inset);

    mesh.visit_triangles(visit_aabb, [&](auto tri_idx) {
        collide_sphere_triangle(sphere, mesh, tri_idx, ctx, result);
    });
}

}
