#include "edyn/constraints/gravity_constraint.hpp"
#include "edyn/dynamics/row_cache.hpp"

namespace edyn {

void gravity_constraint::prepare(
    const entt::registry &, entt::entity,
    constraint_row_prep_cache &cache, scalar dt,
    const constraint_body &bodyA, const constraint_body &bodyB) {

    auto d = bodyA.pos - bodyB.pos;
    auto l2 = length_sqr(d);
    l2 = std::max(l2, EDYN_EPSILON);

    auto l = std::sqrt(l2);
    auto dn = d / l;

    auto F = gravitational_constant / (l2 * bodyA.inv_m * bodyB.inv_m);
    auto P = F * dt;

    auto &row = cache.add_row();
    row.J = {dn, vector3_zero, -dn, -vector3_zero};
    row.lower_limit = -P;
    row.upper_limit = P;
    row.impulse = applied_impulse;

    auto &options = cache.get_options();
    options.error = large_scalar;
}

void gravity_constraint::store_applied_impulses(const std::vector<scalar> &impulses) {
    applied_impulse = impulses[0];
}

}
