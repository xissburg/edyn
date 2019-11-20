#ifndef EDYN_CONSTRAINTS_POINT_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_POINT_CONSTRAINT_HPP

#include <array>
#include <entt/fwd.hpp>
#include "edyn/math/vector3.hpp"

namespace edyn {

struct constraint;
struct relation;

struct point_constraint {
    static constexpr size_t num_rows = 3;

    std::array<vector3, 2> pivot;

    void prepare(constraint *, const relation *, entt::registry &, scalar dt);
};

}

#endif // EDYN_CONSTRAINTS_POINT_CONSTRAINT_HPP