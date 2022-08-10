#ifndef EDYN_CONSTRAINTS_CONTACT_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_CONTACT_CONSTRAINT_HPP

#include <array>
#include <vector>
#include "edyn/math/constants.hpp"
#include "edyn/constraints/constraint_base.hpp"
#include "edyn/constraints/prepare_constraints.hpp"
#include "edyn/util/array_util.hpp"

namespace edyn {

/**
 * @brief Non-penetration constraint.
 */
struct contact_constraint : public constraint_base {
    static constexpr auto num_rows = 4;
    std::array<scalar, num_rows> impulse {make_array<num_rows>(scalar{})};
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
void init_constraints<contact_constraint>(entt::registry &);

template<>
void prepare_constraint<contact_constraint>(const entt::registry &, entt::entity, contact_constraint &con,
                                            constraint_row_prep_cache &cache, scalar dt,
                                            const vector3 &originA,
                                            const vector3 &posA, const quaternion &ornA,
                                            const vector3 &linvelA, const vector3 &angvelA,
                                            scalar inv_mA, const matrix3x3 &inv_IA,
                                            delta_linvel &dvA, delta_angvel &dwA,
                                            const vector3 &originB,
                                            const vector3 &posB, const quaternion &ornB,
                                            const vector3 &linvelB, const vector3 &angvelB,
                                            scalar inv_mB, const matrix3x3 &inv_IB,
                                            delta_linvel &dvB, delta_angvel &dwB);

/* template<>
void iterate_constraints<contact_constraint>(entt::registry &, row_cache &, scalar dt); */

template<>
bool solve_position_constraints<contact_constraint>(entt::registry &registry, scalar dt);

}

#endif // EDYN_CONSTRAINTS_CONTACT_CONSTRAINT_HPP
