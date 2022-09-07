#ifndef EDYN_CONSTRAINTS_CONTACT_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_CONTACT_CONSTRAINT_HPP

#include "edyn/config/constants.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/constraints/constraint_base.hpp"
#include "edyn/constraints/constraint_body.hpp"
#include "edyn/util/array_util.hpp"

namespace edyn {

class position_solver;
struct constraint_row_prep_cache;
struct contact_manifold;

/**
 * @brief Non-penetration constraint.
 */
struct contact_constraint : public constraint_base {
    static constexpr auto num_rows = max_contacts + // Non-penetration
                                     max_contacts * 2 + // Friction
                                     max_contacts * 2 + // Rolling resistance
                                     max_contacts; // Spinning friction
    std::array<scalar, num_rows> impulse {make_array<num_rows>(scalar{})};

    void prepare(
        const entt::registry &, entt::entity, const contact_manifold &,
        constraint_row_prep_cache &cache, scalar dt,
        const constraint_body &bodyA, const constraint_body &bodyB);

    void solve_position(position_solver &, contact_manifold &);
};

template<typename Archive>
void serialize(Archive &archive, contact_constraint &c) {
    archive(c.body);
    archive(c.impulse);
}

}

#endif // EDYN_CONSTRAINTS_CONTACT_CONSTRAINT_HPP
