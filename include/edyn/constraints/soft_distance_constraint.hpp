#ifndef EDYN_CONSTRAINTS_SOFT_DISTANCE_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_SOFT_DISTANCE_CONSTRAINT_HPP

#include <array>
#include <entt/fwd.hpp>
#include "edyn/math/vector3.hpp"
#include "edyn/constraints/constraint_base.hpp"
#include "edyn/constraints/prepare_constraints.hpp"

namespace edyn {

struct soft_distance_constraint : public constraint_base {
    std::array<vector3, 2> pivot;
    scalar distance {0};
    scalar stiffness {1e10};
    scalar damping {1e10};

    scalar m_relspd {};
};

template<>
void prepare_constraints<soft_distance_constraint>(entt::registry &, row_cache &, scalar dt);

template<>
void iterate_constraints<soft_distance_constraint>(entt::registry &, row_cache &, scalar dt);

}

#endif // EDYN_CONSTRAINTS_SOFT_DISTANCE_CONSTRAINT_HPP
