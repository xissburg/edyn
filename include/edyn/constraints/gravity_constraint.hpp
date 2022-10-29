#ifndef EDYN_CONSTRAINTS_GRAVITY_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_GRAVITY_CONSTRAINT_HPP

#include <vector>
#include <entt/entity/fwd.hpp>
#include "edyn/math/scalar.hpp"
#include "edyn/constraints/constraint_base.hpp"
#include "edyn/constraints/constraint_body.hpp"

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
    scalar applied_impulse {scalar(0)};

    void prepare(
        const entt::registry &, entt::entity,
        constraint_row_prep_cache &cache, scalar dt,
        const constraint_body &bodyA, const constraint_body &bodyB);

    void store_applied_impulses(const std::vector<scalar> &impulses);
};

template<typename Archive>
void serialize(Archive &archive, gravity_constraint &con) {
    archive(con.body);
    archive(con.applied_impulse);
}


}

#endif // EDYN_CONSTRAINTS_GRAVITY_CONSTRAINT_HPP
