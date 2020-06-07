#ifndef EDYN_COLLISION_CONTACT_POINT_HPP
#define EDYN_COLLISION_CONTACT_POINT_HPP

#include <entt/fwd.hpp>
#include <cstdint>
#include "edyn/math/vector3.hpp"

namespace edyn {

struct contact_point {
    entt::entity parent;
    vector3 pivotA;
    vector3 pivotB;
    vector3 normalB;
    scalar friction;
    scalar restitution;
    uint32_t lifetime {0};
    scalar distance;
    scalar impulse;
};

}

#endif // EDYN_COLLISION_CONTACT_POINT_HPP