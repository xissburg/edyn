#ifndef EDYN_CONSTRAINTS_GRAVITY_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_GRAVITY_CONSTRAINT_HPP

#include <entt/fwd.hpp>
#include "edyn/math/vector3.hpp"
#include "edyn/constraints/constraint_base.hpp"
#include "edyn/constraints/prepare_constraints.hpp"

namespace edyn {

struct gravity_constraint : public constraint_base {

};

template<>
void prepare_constraints<gravity_constraint>(entt::registry &, row_cache &, scalar dt);

}

#endif // EDYN_CONSTRAINTS_GRAVITY_CONSTRAINT_HPP
