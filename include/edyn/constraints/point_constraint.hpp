#ifndef EDYN_CONSTRAINTS_POINT_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_POINT_CONSTRAINT_HPP

#include <array>
#include <entt/fwd.hpp>
#include "edyn/math/vector3.hpp"

namespace edyn {

class row_cache;
struct constraint;

struct point_constraint {
    std::array<vector3, 2> pivot;

    void prepare(entt::entity, const constraint &, entt::registry &, row_cache &cache, scalar dt);
};

}

#endif // EDYN_CONSTRAINTS_POINT_CONSTRAINT_HPP