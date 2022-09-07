#include "edyn/constraints/gravity_constraint.hpp"
#include "edyn/dynamics/row_cache.hpp"

namespace edyn {

void gravity_constraint::prepare(
    const entt::registry &, entt::entity,
    constraint_row_prep_cache &cache, scalar dt,
    const vector3 &originA, const vector3 &posA, const quaternion &ornA,
    const vector3 &linvelA, const vector3 &angvelA,
    scalar inv_mA, const matrix3x3 &inv_IA,
    const vector3 &originB, const vector3 &posB, const quaternion &ornB,
    const vector3 &linvelB, const vector3 &angvelB,
    scalar inv_mB, const matrix3x3 &inv_IB) {

    auto d = posA - posB;
    auto l2 = length_sqr(d);
    l2 = std::max(l2, EDYN_EPSILON);

    auto l = std::sqrt(l2);
    auto dn = d / l;

    auto F = gravitational_constant / (l2 * inv_mA * inv_mB);
    auto P = F * dt;

    auto &row = cache.add_row();
    row.J = {dn, vector3_zero, -dn, -vector3_zero};
    row.lower_limit = -P;
    row.upper_limit = P;
    row.impulse = impulse;

    auto &options = cache.get_options();
    options.error = large_scalar;
}

}
