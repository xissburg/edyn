#ifndef EDYN_CONSTRAINTS_CONTACT_EXTRAS_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_CONTACT_EXTRAS_CONSTRAINT_HPP

#include "edyn/comp/roll_direction.hpp"
#include "edyn/constraints/contact_constraint.hpp"

namespace edyn {

/**
 * @brief A special case of contact_constraint with extra features.
 */
struct contact_extras_constraint : public contact_constraint {
    scalar stiffness;
    scalar damping;

    scalar spin_friction;
    scalar roll_friction;
    std::array<roll_direction, 2> roll_dir;

    scalar spin_friction_impulse; // Applied spin friction impulse.
    std::array<scalar, 2> rolling_friction_impulse; // Applied rolling friction impulse.

    void prepare(constraint_row_prep_cache &cache, scalar dt,
                 const constraint_body &bodyA, const constraint_body &bodyB);

    void solve_position(position_solver &solver);

    void store_applied_impulses(const std::vector<scalar> &impulses);
};

}

#endif // EDYN_CONSTRAINTS_CONTACT_EXTRAS_CONSTRAINT_HPP
