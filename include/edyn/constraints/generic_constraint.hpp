#ifndef EDYN_CONSTRAINTS_GENERIC_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_GENERIC_CONSTRAINT_HPP

#include <array>
#include <entt/entity/fwd.hpp>
#include "edyn/math/vector3.hpp"
#include "edyn/constraints/constraint_base.hpp"
#include "edyn/constraints/prepare_constraints.hpp"
#include "edyn/util/array.hpp"

namespace edyn {

struct generic_constraint : public constraint_base {
    std::array<vector3, 2> pivot;
    std::array<scalar, 6> impulse {make_array<6, scalar>(0)};
};

template<>
void prepare_constraints<generic_constraint>(entt::registry &, row_cache &, scalar dt);

}

#endif // EDYN_CONSTRAINTS_GENERIC_CONSTRAINT_HPP
