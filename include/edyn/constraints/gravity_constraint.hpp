#ifndef EDYN_CONSTRAINTS_GRAVITY_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_GRAVITY_CONSTRAINT_HPP

#include <entt/entity/fwd.hpp>
#include "edyn/math/scalar.hpp"
#include "edyn/constraints/constraint_base.hpp"

namespace edyn {

struct vector3;
struct quaternion;
struct matrix3x3;
class position_solver;
struct constraint_row_prep_cache;

/**
 * @brief Applies gravitational attraction forces between two entities.
 */
struct gravity_constraint : public constraint_base {
    scalar impulse {scalar(0)};

    void prepare(
        const entt::registry &, entt::entity,
        constraint_row_prep_cache &cache, scalar dt,
        const vector3 &originA, const vector3 &posA, const quaternion &ornA,
        const vector3 &linvelA, const vector3 &angvelA,
        scalar inv_mA, const matrix3x3 &inv_IA,
        const vector3 &originB, const vector3 &posB, const quaternion &ornB,
        const vector3 &linvelB, const vector3 &angvelB,
        scalar inv_mB, const matrix3x3 &inv_IB);
};

template<typename Archive>
void serialize(Archive &archive, gravity_constraint &con) {
    archive(con.body);
    archive(con.impulse);
}


}

#endif // EDYN_CONSTRAINTS_GRAVITY_CONSTRAINT_HPP
