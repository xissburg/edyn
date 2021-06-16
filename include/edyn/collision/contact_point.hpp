#ifndef EDYN_COLLISION_CONTACT_POINT_HPP
#define EDYN_COLLISION_CONTACT_POINT_HPP

#include <array>
#include <cstdint>
#include <entt/entity/fwd.hpp>
#include <entt/entity/entity.hpp>
#include "edyn/math/vector3.hpp"

namespace edyn {

struct contact_point {
    std::array<entt::entity, 2> body {entt::null, entt::null};
    vector3 pivotA;
    vector3 pivotB;
    vector3 normalB;
    scalar friction;
    scalar restitution;
    uint32_t lifetime {0};
    scalar distance;
};

}

#endif // EDYN_COLLISION_CONTACT_POINT_HPP
