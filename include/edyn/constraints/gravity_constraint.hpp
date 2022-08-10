#ifndef EDYN_CONSTRAINTS_GRAVITY_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_GRAVITY_CONSTRAINT_HPP

#include <entt/entity/fwd.hpp>
#include "edyn/math/vector3.hpp"
#include "edyn/constraints/constraint_base.hpp"
#include "edyn/constraints/prepare_constraints.hpp"

namespace edyn {

/**
 * @brief Applies gravitational attraction forces between two entities.
 */
struct gravity_constraint : public constraint_base {
    scalar impulse {scalar(0)};
};

template<typename Archive>
void serialize(Archive &archive, gravity_constraint &con) {
    archive(con.body);
    archive(con.impulse);
}

template<>
void prepare_constraint<gravity_constraint>(const entt::registry &, entt::entity, gravity_constraint &con,
                                            constraint_row_prep_cache &cache, scalar dt,
                                            const vector3 &originA, const vector3
                                            &posA, const quaternion &ornA,
                                            const vector3 &linvelA, const vector3 &angvelA,
                                            scalar inv_mA, const matrix3x3 &inv_IA,
                                            delta_linvel &dvA, delta_angvel &dwA,
                                            const vector3 &originB,
                                            const vector3 &posB, const quaternion &ornB,
                                            const vector3 &linvelB, const vector3 &angvelB,
                                            scalar inv_mB, const matrix3x3 &inv_IB,
                                            delta_linvel &dvB, delta_angvel &dwB);

}

#endif // EDYN_CONSTRAINTS_GRAVITY_CONSTRAINT_HPP
