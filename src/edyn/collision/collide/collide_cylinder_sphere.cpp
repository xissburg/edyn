#include "edyn/collision/collide.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/math.hpp"

namespace edyn {

void collide(const cylinder_shape &shA, const sphere_shape &shB,
             const collision_context &ctx, collision_result &result) {
    const auto &posA = ctx.posA;
    const auto &ornA = ctx.ornA;
    const auto &posB = ctx.posB;
    const auto &ornB = ctx.ornB;
    const auto threshold = ctx.threshold;

    const auto cyl_axis = quaternion_x(ornA);
    const auto hl = cyl_axis * shA.half_length;
    const auto p0 = posA - hl;
    const auto p1 = posA + hl;

    const auto v = p1 - p0; // Direction vector of cylinder segment.
    const auto w = posB - p0; // Vector from initial point of cylinder segment to point sphere center.
    const auto a = dot(w, v);
    const auto b = dot(v, v);
    EDYN_ASSERT(b > EDYN_EPSILON);
    const auto t = a / b;

    if (t > 0 && t < 1) {
        const auto p = p0 + v * t;
        const auto d = p - posB;
        const auto l2 = length_sqr(d);
        const auto min_dist = shA.radius + shB.radius + threshold;

        if (l2 > min_dist * min_dist) {
            return;
        }

        const auto l = std::sqrt(l2);
        const auto normal = l2 > EDYN_EPSILON ? d / l : vector3_y;

        auto pivotA = rotate(conjugate(ornA), p - normal * shA.radius - posA);
        auto pivotB = rotate(conjugate(ornB), normal * shB.radius);
        auto distance = l - shA.radius - shB.radius;
        result.add_point({pivotA, pivotB, normal, distance, contact_normal_attachment::none});
        return;
    }

    const auto dpos = t < 0.5 ? p0 : p1;
    vector3 q;
    const auto l2 = closest_point_disc(dpos, ornA, shA.radius, posB, q);
    const auto min_dist = shB.radius + threshold;

    if (l2 > min_dist * min_dist) {
        return;
    }

    auto normal = q - posB;
    const auto nl2 = length_sqr(normal);
    const auto nl = std::sqrt(nl2);
    normal = nl2 > EDYN_EPSILON ? normal / nl : rotate(ornA, vector3_x) * to_sign(t > 0.5);

    auto pivotA = rotate(conjugate(ornA), q - posA);
    auto pivotB = rotate(conjugate(ornB), normal * shB.radius);
    auto distance = nl - shB.radius;

    // If the closest feature is the cylinder face, attach normal to A.
    // I.e. if the projection of the sphere center is inside the cap face.
    auto normal_attachment = contact_normal_attachment::none;
    auto sphere_proj = project_plane(posB, posA, cyl_axis);

    if (distance_sqr(sphere_proj, posA) < shA.radius * shA.radius) {
        normal_attachment = contact_normal_attachment::normal_on_A;
    }

    result.add_point({pivotA, pivotB, normal, distance, normal_attachment});
}

void collide(const sphere_shape &shA, const cylinder_shape &shB,
             const collision_context &ctx, collision_result &result) {
    swap_collide(shA, shB, ctx, result);
}

}
