#include "edyn/constraints/soft_distance_constraint.hpp"
#include "edyn/dynamics/row_cache.hpp"
#include "edyn/math/transform.hpp"
#include "edyn/constraints/constraint_body.hpp"

namespace edyn {

void soft_distance_constraint::prepare(
    const entt::registry &, entt::entity,
    constraint_row_prep_cache &cache, scalar dt,
    const constraint_body &bodyA, const constraint_body &bodyB) {

    auto pivotA = to_world_space(pivot[0], bodyA.origin, bodyA.orn);
    auto pivotB = to_world_space(pivot[1], bodyB.origin, bodyB.orn);
    auto rA = pivotA - bodyA.pos;
    auto rB = pivotB - bodyB.pos;
    auto d = pivotA - pivotB;
    auto dist_sqr = length_sqr(d);
    auto dist = std::sqrt(dist_sqr);
    vector3 dn;

    if (dist_sqr > EDYN_EPSILON) {
        dn = d / dist;
    } else {
        d = dn = vector3_x;
    }

    auto p = cross(rA, dn);
    auto q = cross(rB, dn);

    {
        // Spring row. By setting the error to +/- `large_scalar`, it will
        // always apply the impulse set in the limits.
        auto error = distance - dist;
        auto spring_force = stiffness * error;
        auto spring_impulse = spring_force * dt;

        auto &row = cache.add_row();
        row.J = {dn, p, -dn, -q};
        row.lower_limit = std::min(spring_impulse, scalar(0));
        row.upper_limit = std::max(scalar(0), spring_impulse);
        row.impulse = applied_spring_impulse;

        auto &options = cache.get_options();
        options.error = spring_impulse > 0 ? -large_scalar : large_scalar;
    }

    {
        // Damping row. It functions like friction where the force is
        // proportional to the relative speed.
        auto &row = cache.add_row();
        row.J = {dn, p, -dn, -q};

        auto relspd = dot(row.J[0], bodyA.linvel) +
                      dot(row.J[1], bodyA.angvel) +
                      dot(row.J[2], bodyB.linvel) +
                      dot(row.J[3], bodyB.angvel);
        auto damping_force = damping * relspd;
        auto damping_impulse = damping_force * dt;
        row.lower_limit = -std::abs(damping_impulse);
        row.upper_limit =  std::abs(damping_impulse);
        row.impulse = applied_damping_impulse;
    }
}

void soft_distance_constraint::store_applied_impulses(const std::vector<scalar> &impulses) {
    applied_spring_impulse = impulses[0];
    applied_damping_impulse = impulses[1];
}

}
