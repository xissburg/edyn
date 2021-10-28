#ifndef EDYN_CONSTRAINTS_CONTACT_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_CONTACT_CONSTRAINT_HPP

#include <vector>
#include <entt/entity/fwd.hpp>
#include "edyn/math/constants.hpp"
#include "edyn/constraints/constraint_base.hpp"
#include "edyn/constraints/prepare_constraints.hpp"

namespace edyn {

/**
 * @brief Non-penetration constraint.
 * The corresponding `constraint_impulse` holds the applied impulses in the
 * following order:
 * 0 - normal
 * 1 - first tangent friction
 * 2 - second tangent friction
 * 3 - spinning friction
 * 4 - first rolling friction
 * 5 - second rolling friction
 * 6 - normal restitution impulse
 * 7 - first restitution tangent friction
 * 8 - second restitution tangent friction
 * The restitution impulses are calculated by the restitution solver and are
 * kept separate because if mixed with the values at [0, 2], the constraint
 * solver will remove some of the propagated shock and the results will not
 * be correct. Add them up to get the full normal and friction impulses.
 */
struct contact_constraint : public constraint_base {
    scalar stiffness {large_scalar};
    scalar damping {large_scalar};
};

struct constraint_row;

namespace internal {
    struct contact_friction_row {
        std::array<vector3, 4> J;
        scalar eff_mass;
        scalar rhs;
        scalar impulse;
    };

    struct contact_friction_row_pair {
        contact_friction_row row[2];
        scalar friction_coefficient;
    };

    struct contact_constraint_context {
        std::vector<contact_friction_row_pair> friction_rows;
        std::vector<contact_friction_row_pair> roll_friction_rows;
        size_t row_start_index;
        size_t row_count_start_index;
    };

    void solve_friction_row_pair(internal::contact_friction_row_pair &friction_row_pair, constraint_row &normal_row);
}

template<>
void prepare_constraints<contact_constraint>(entt::registry &, row_cache &, scalar dt);

template<>
void iterate_constraints<contact_constraint>(entt::registry &, row_cache &, scalar dt);

template<>
bool solve_position_constraints<contact_constraint>(entt::registry &registry, scalar dt);

}

#endif // EDYN_CONSTRAINTS_CONTACT_CONSTRAINT_HPP
