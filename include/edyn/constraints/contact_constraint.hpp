#ifndef EDYN_CONSTRAINTS_CONTACT_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_CONTACT_CONSTRAINT_HPP

#include <array>
#include <vector>
#include "edyn/config/constants.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/constraints/constraint_base.hpp"
#include "edyn/constraints/prepare_constraints.hpp"
#include "edyn/util/array_util.hpp"

namespace edyn {

/**
 * @brief Non-penetration constraint.
 */
struct contact_constraint : public constraint_base {
    static constexpr auto num_rows = max_contacts + // Non-penetration
                                     max_contacts * 2 + // Friction
                                     max_contacts * 2 + // Rolling resistance
                                     max_contacts; // Spinning friction
    std::array<scalar, num_rows> impulse {make_array<num_rows>(scalar{})};
};

template<typename Archive>
void serialize(Archive &archive, contact_constraint &c) {
    archive(c.body);
    archive(c.impulse);
}

template<>
void prepare_constraint<contact_constraint>(
    const entt::registry &, entt::entity, contact_constraint &con,
    constraint_row_prep_cache &cache, scalar dt,
    const vector3 &originA, const vector3 &posA, const quaternion &ornA,
    const vector3 &linvelA, const vector3 &angvelA,
    scalar inv_mA, const matrix3x3 &inv_IA,
    const vector3 &originB, const vector3 &posB, const quaternion &ornB,
    const vector3 &linvelB, const vector3 &angvelB,
    scalar inv_mB, const matrix3x3 &inv_IB);

template<>
void prepare_position_constraint<contact_constraint>(
    entt::registry &registry, entt::entity entity, contact_constraint &con,
    position_solver &solver);

}

#endif // EDYN_CONSTRAINTS_CONTACT_CONSTRAINT_HPP
