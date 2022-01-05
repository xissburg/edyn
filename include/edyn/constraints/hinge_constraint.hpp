#ifndef EDYN_CONSTRAINTS_HINGE_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_HINGE_CONSTRAINT_HPP

#include <array>
#include <entt/entity/fwd.hpp>
#include "edyn/math/vector3.hpp"
#include "edyn/constraints/constraint_base.hpp"
#include "edyn/constraints/prepare_constraints.hpp"
#include "edyn/util/array.hpp"

namespace edyn {

struct hinge_constraint : public constraint_base {
    // Pivots in object space.
    std::array<vector3, 2> pivot;
    // Rotation axes in object space.
    std::array<vector3, 2> axis;
    std::array<scalar, 5> impulse {make_array<5, scalar>(0)};
};

template<>
void prepare_constraints<hinge_constraint>(entt::registry &, row_cache &, scalar dt);


template<>
bool solve_position_constraints<hinge_constraint>(entt::registry &, scalar dt);

}

#endif // EDYN_CONSTRAINTS_HINGE_CONSTRAINT_HPP
