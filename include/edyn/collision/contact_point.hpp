#ifndef EDYN_COLLISION_CONTACT_POINT_HPP
#define EDYN_COLLISION_CONTACT_POINT_HPP

#include <entt/entt.hpp>
#include "edyn/math/vector3.hpp"

namespace edyn {

struct contact_point {
    vector3 pivotA;
    vector3 pivotB;
    vector3 normalB;
    scalar friction;
    scalar restitution;
    scalar restitution_multiplier {1};
    entt::entity normal_row_entity {entt::null};
    entt::entity friction_row_entity {entt::null};
};

}

#endif // EDYN_COLLISION_CONTACT_POINT_HPP