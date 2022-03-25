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
void prepare_constraints<gravity_constraint>(entt::registry &, row_cache &, scalar dt);

}

#endif // EDYN_CONSTRAINTS_GRAVITY_CONSTRAINT_HPP
