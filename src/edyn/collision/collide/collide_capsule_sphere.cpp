#include "edyn/collision/collide.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/collision/collision_result.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/transform.hpp"

namespace edyn {

void collide(const capsule_shape &shA, const sphere_shape &shB,
             const collision_context &ctx, collision_result &result) {
    auto capsule_vertices = shA.get_vertices(ctx.posA, ctx.ornA);
    vector3 closest; scalar t;
    auto dist_sqr = closest_point_segment(capsule_vertices[0], capsule_vertices[1], ctx.posB, t, closest);
    auto min_dist = shA.radius + shB.radius + ctx.threshold;

    if (dist_sqr > min_dist * min_dist) {
        return;
    }

    auto normal = closest - ctx.posB;
    auto normal_len_sqr = length_sqr(normal);
    scalar distance;

    if (normal_len_sqr > EDYN_EPSILON) {
        auto normal_len = std::sqrt(normal_len_sqr);
        normal /= normal_len;
        distance = normal_len - shA.radius - shB.radius;
    } else {
        // Pick a direction orthogonal to the capsule's axis.
        normal = quaternion_z(ctx.ornA);
        distance = -(shA.radius + shB.radius);
    }

    collision_result::collision_point point;
    auto normalB = rotate(conjugate(ctx.ornB), normal);
    auto pivotA_world = closest - normal * shA.radius;
    point.pivotA = to_object_space(pivotA_world, ctx.posA, ctx.ornA);
    point.pivotB = normalB * shB.radius;
    point.normal = normal;
    point.distance = distance;
    point.normal_attachment = contact_normal_attachment::none;

    if (t > 0 && t < 1) {
        point.featureA = {capsule_feature::side};
    } else {
        point.featureA = {capsule_feature::hemisphere};
        point.featureA->index = t == 0 ? 0 : 1;
    }

    result.add_point(point);
}

void collide(const sphere_shape &shA, const capsule_shape &shB,
             const collision_context &ctx, collision_result &result) {
    swap_collide(shA, shB, ctx, result);
}

}
