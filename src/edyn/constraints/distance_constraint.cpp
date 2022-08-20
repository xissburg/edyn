#include "edyn/constraints/distance_constraint.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/math/transform.hpp"
#include <entt/entity/registry.hpp>

namespace edyn {

void distance_constraint::prepare(
    const entt::registry &, entt::entity,
    constraint_row_prep_cache &cache, scalar dt,
    const vector3 &originA, const vector3 &posA, const quaternion &ornA,
    const vector3 &linvelA, const vector3 &angvelA,
    scalar inv_mA, const matrix3x3 &inv_IA,
    const vector3 &originB, const vector3 &posB, const quaternion &ornB,
    const vector3 &linvelB, const vector3 &angvelB,
    scalar inv_mB, const matrix3x3 &inv_IB) {

    auto pivotA = to_world_space(pivot[0], originA, ornA);
    auto pivotB = to_world_space(pivot[1], originB, ornB);
    auto rA = pivotA - posA;
    auto rB = pivotB - posB;

    auto d = pivotA - pivotB;
    auto dist_sqr = length_sqr(d);

    if (!(dist_sqr > EDYN_EPSILON)) {
        d = vector3_x;
    }

    auto &row = cache.add_row();
    row.J = {d, cross(rA, d), -d, -cross(rB, d)};
    row.lower_limit = -large_scalar;
    row.upper_limit =  large_scalar;
    row.impulse = impulse;

    auto &options = cache.get_options();
    options.error = scalar(0.5) * (dist_sqr - distance * distance) / dt;
}

}
