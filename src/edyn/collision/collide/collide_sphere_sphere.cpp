#include "edyn/collision/collide.hpp"

namespace edyn {

collision_result collide(const sphere_shape &shA, const sphere_shape &shB, 
                         const collision_context &ctx) {
    auto d = ctx.posA - ctx.posB;
    auto l2 = length_sqr(d);
    auto r = shA.radius + shB.radius + ctx.threshold;

    if (l2 > r * r) {
        return {};
    }

    auto l = std::sqrt(l2);
    auto dn = d / l;
    auto rA = -dn * shA.radius;
    rA = rotate(conjugate(ctx.ornA), rA);
    auto rB = dn * shB.radius;
    rB = rotate(conjugate(ctx.ornB), rB);

    auto result = collision_result {};
    result.num_points = 1;
    result.point[0].pivotA = rA;
    result.point[0].pivotB = rB;
    result.point[0].normalB = rotate(conjugate(ctx.ornB), dn);
    result.point[0].distance = l - shA.radius - shB.radius;
    return result;
}

}
