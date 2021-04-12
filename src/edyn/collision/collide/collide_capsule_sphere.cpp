#include "edyn/collision/collide.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/collision/collision_result.hpp"
#include "edyn/math/geom.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/scalar.hpp"

namespace edyn {

collision_result collide(const capsule_shape &shA, const sphere_shape &shB, 
                         const collision_context &ctx) {
    auto capsule_vertices = shA.get_vertices(ctx.posA, ctx.ornA);
    vector3 closest; scalar t;
    auto dist_sqr = closest_point_segment(capsule_vertices[0], capsule_vertices[1], ctx.posB, t, closest);
    auto min_dist = shA.radius + shB.radius + ctx.threshold;

    if (dist_sqr > min_dist * min_dist) {
        return {};
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

    auto normalB = rotate(conjugate(ctx.ornB), normal);
    auto pivotB = normalB * shB.radius;
    auto pivotA_world = closest - normal * shA.radius;
    auto pivotA = to_object_space(pivotA_world, ctx.posA, ctx.ornA);

    auto result = collision_result{};
    result.add_point({pivotA, pivotB, normalB, distance});
    return result;
}

collision_result collide(const sphere_shape &shA, const capsule_shape &shB,
                         const collision_context &ctx) {
    return swap_collide(shA, shB, ctx);
}

}
