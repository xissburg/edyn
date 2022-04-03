#ifndef EDYN_CONSTRAINTS_CONTACT_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_CONTACT_CONSTRAINT_HPP

#include <array>
#include <vector>
#include "edyn/math/constants.hpp"
#include "edyn/constraints/constraint_base.hpp"
#include "edyn/constraints/prepare_constraints.hpp"

namespace edyn {

/**
 * @brief Non-penetration constraint.
 */
struct contact_constraint : public constraint_base {

};

template<typename Archive>
void serialize(Archive &archive, contact_constraint &c) {
    archive(c.body);
}

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

        /**
         * Index where the contact constraints start in the row cache, i.e.
         * `row_cache::rows`.
         */
        size_t row_start_index;

        /**
         * Index where the contact constraints start in the row count array,
         * i.e. `row_cache::con_num_rows`.
         */
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
