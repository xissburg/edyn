#ifndef EDYN_CONSTRAINTS_POINT_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_POINT_CONSTRAINT_HPP

#include <array>
#include <entt/fwd.hpp>
#include "edyn/math/vector3.hpp"
#include "edyn/constraints/constraint_base.hpp"

namespace edyn {

struct row_cache;

struct point_constraint : public constraint_base {
    std::array<vector3, 2> pivot;
};

void prepare_point_constraints(entt::registry &, row_cache &, scalar dt);

}

#endif // EDYN_CONSTRAINTS_POINT_CONSTRAINT_HPP
