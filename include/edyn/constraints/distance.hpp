#ifndef EDYN_CONSTRAINTS_DISTANCE_HPP
#define EDYN_CONSTRAINTS_DISTANCE_HPP

#include <array>
#include <entt/entt.hpp>
#include "edyn/math/vector3.hpp"

namespace edyn {

struct distance_constraint {
    std::array<entt::entity, 2> entity;
    std::array<vector3, 2> pivot;
    scalar distance {0};
};

}


#endif // EDYN_CONSTRAINTS_DISTANCE_HPP