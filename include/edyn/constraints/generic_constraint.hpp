#ifndef EDYN_CONSTRAINTS_GENERIC_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_GENERIC_CONSTRAINT_HPP

#include <array>
#include <entt/fwd.hpp>
#include "edyn/math/vector3.hpp"
#include "edyn/constraints/constraint_base.hpp"

namespace edyn {

struct row_cache;

struct generic_constraint : public constraint_base {
    std::array<vector3, 2> pivot;
};

void prepare_generic_constraints(entt::registry &, row_cache &, scalar dt);

}

#endif // EDYN_CONSTRAINTS_GENERIC_CONSTRAINT_HPP
