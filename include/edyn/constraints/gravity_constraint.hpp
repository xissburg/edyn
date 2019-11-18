#ifndef EDYN_CONSTRAINTS_GRAVITY_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_GRAVITY_CONSTRAINT_HPP

#include <entt/fwd.hpp>
#include "edyn/math/vector3.hpp"

namespace edyn {

struct constraint;

struct gravity_constraint {
    static constexpr size_t num_rows = 1;

    void prepare(constraint *, entt::registry &, scalar dt);
};

}

#endif // EDYN_CONSTRAINTS_GRAVITY_CONSTRAINT_HPP