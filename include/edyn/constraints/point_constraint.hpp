#ifndef EDYN_CONSTRAINTS_POINT_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_POINT_CONSTRAINT_HPP

#include <array>
#include <entt/fwd.hpp>
#include "edyn/math/vector3.hpp"

namespace edyn {

struct constraint;
struct relation;

struct point_constraint {
    std::array<vector3, 2> pivot;

    void init(constraint *, const relation *, entt::registry &);
    void prepare(constraint *, const relation *, entt::registry &, scalar dt);
    void before_solve(constraint *, const relation *, entt::registry &, scalar dt);
};

}

#endif // EDYN_CONSTRAINTS_POINT_CONSTRAINT_HPP