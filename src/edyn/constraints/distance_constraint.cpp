#include "edyn/constraints/distance_constraint.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/math/transform.hpp"

namespace edyn {

void distance_constraint::prepare(
    const entt::registry &, entt::entity,
    constraint_row_prep_cache &cache, scalar dt,
    const constraint_body &bodyA, const constraint_body &bodyB) {

    auto pivotA = to_world_space(pivot[0], bodyA.origin, bodyA.orn);
    auto pivotB = to_world_space(pivot[1], bodyB.origin, bodyB.orn);
    auto rA = pivotA - bodyA.pos;
    auto rB = pivotB - bodyB.pos;

    auto d = pivotA - pivotB;
    auto dist_sqr = length_sqr(d);

    if (!(dist_sqr > EDYN_EPSILON)) {
        d = vector3_x;
    }

    auto &row = cache.add_row();
    row.J = {d, cross(rA, d), -d, -cross(rB, d)};
    row.lower_limit = -large_scalar;
    row.upper_limit =  large_scalar;
    row.impulse = applied_impulse;

    auto &options = cache.get_options();
    options.error = scalar(0.5) * (dist_sqr - distance * distance) / dt;
}

void distance_constraint::store_applied_impulses(const std::vector<scalar> &impulses) {
    applied_impulse = impulses[0];
}

}
