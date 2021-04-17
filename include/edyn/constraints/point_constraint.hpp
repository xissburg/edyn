#ifndef EDYN_CONSTRAINTS_POINT_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_POINT_CONSTRAINT_HPP

#include <array>
#include <entt/fwd.hpp>
#include "edyn/math/vector3.hpp"
#include "edyn/constraints/constraint_base.hpp"
#include "edyn/constraints/prepare_constraints.hpp"

namespace edyn {

struct point_constraint : public constraint_base {
    std::array<vector3, 2> pivot;
};

template<>
void prepare_constraints<point_constraint>(entt::registry &, row_cache &, scalar dt);

}

#endif // EDYN_CONSTRAINTS_POINT_CONSTRAINT_HPP
