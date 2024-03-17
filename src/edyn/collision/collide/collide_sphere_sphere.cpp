#include "edyn/collision/collide.hpp"

namespace edyn {

void collide(const sphere_shape &shA, const sphere_shape &shB,
             const collision_context &ctx, collision_result &result) {
    auto d = ctx.posA - ctx.posB;
    auto dist_sqr = length_sqr(d);
    auto r = shA.radius + shB.radius + ctx.threshold;

    if (dist_sqr > r * r) {
        return;
    }

    auto dist = std::sqrt(dist_sqr);
    auto dn = dist > EDYN_EPSILON ? d / dist : vector3_x;
    auto rA = -dn * shA.radius;
    rA = rotate(conjugate(ctx.ornA), rA);
    auto rB = dn * shB.radius;
    rB = rotate(conjugate(ctx.ornB), rB);

    auto pivotA = rA;
    auto pivotB = rB;
    auto normal = dn;
    auto distance = dist - shA.radius - shB.radius;
    result.add_point({pivotA, pivotB, normal, distance, contact_normal_attachment::none});
}

}
