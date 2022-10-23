#ifndef EDYN_CONSTRAINTS_CONTACT_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_CONTACT_CONSTRAINT_HPP

#include "edyn/config/constants.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/constraints/constraint_base.hpp"
#include "edyn/constraints/constraint_body.hpp"

namespace edyn {

class position_solver;
struct constraint_row_prep_cache;
struct contact_manifold;

/**
 * @brief Non-penetration constraint.
 */
struct contact_constraint : public constraint_base {
    void prepare(
        const entt::registry &, entt::entity, const contact_manifold &,
        constraint_row_prep_cache &cache, scalar dt,
        const constraint_body &bodyA, const constraint_body &bodyB);

    void solve_position(position_solver &, contact_manifold &);

    void store_applied_impulse(scalar impulse, unsigned row_index, contact_manifold &);
    void store_friction_impulse(scalar impulse0, scalar impulse1, unsigned row_index, contact_manifold &);
    void store_rolling_impulse(scalar impulse0, scalar impulse1, unsigned row_index, contact_manifold &);
    void store_spinning_impulse(scalar impulse, unsigned row_index, contact_manifold &);
};

template<typename Archive>
void serialize(Archive &archive, contact_constraint &c) {
    archive(c.body);
}

}

#endif // EDYN_CONSTRAINTS_CONTACT_CONSTRAINT_HPP
