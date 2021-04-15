#ifndef EDYN_CONSTRAINTS_DISTANCE_CONSTRAINT
#define EDYN_CONSTRAINTS_DISTANCE_CONSTRAINT

#include <array>
#include <entt/fwd.hpp>
#include "edyn/math/vector3.hpp"

namespace edyn {

class row_cache;
struct constraint;

struct distance_constraint  {
    std::array<vector3, 2> pivot;
    scalar distance {0};

    void prepare(entt::entity, const constraint &, entt::registry &, row_cache &cache, scalar dt);
};

}

#endif // EDYN_CONSTRAINTS_DISTANCE_CONSTRAINT