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
        const entt::registry &, entt::entity,
        constraint_row_prep_cache &cache, scalar dt,
        const constraint_body &bodyA, const constraint_body &bodyB);

    void solve_position(entt::registry &, entt::entity, position_solver &);
};

template<typename Archive>
void serialize(Archive &archive, contact_constraint &c) {
    archive(c.body);
}

}

#endif // EDYN_CONSTRAINTS_CONTACT_CONSTRAINT_HPP
