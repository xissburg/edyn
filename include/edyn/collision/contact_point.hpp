#ifndef EDYN_COLLISION_CONTACT_POINT_HPP
#define EDYN_COLLISION_CONTACT_POINT_HPP

#include <array>
#include <cstdint>
#include <optional>
#include <entt/entity/fwd.hpp>
#include <entt/entity/entity.hpp>
#include "edyn/math/vector3.hpp"
#include "edyn/collision/contact_normal_attachment.hpp"
#include "edyn/collision/collision_feature.hpp"

namespace edyn {

struct contact_point {
    std::array<entt::entity, 2> body {entt::null, entt::null};
    vector3 pivotA; // A's pivot in object space.
    vector3 pivotB; // B's pivot in object space.
    vector3 normal; // Normal in world space.
    vector3 local_normal; // Normal in object space.
    contact_normal_attachment normal_attachment; // To which body the normal is attached.
    scalar friction; // Combined friction coefficient.
    scalar spin_friction; // Combined spin friction coefficient.
    scalar roll_friction; // Combined rolling friction coefficient.
    scalar restitution; // Combined coefficient of restitution.
    uint32_t lifetime {0}; // Incremented in each simulation step where the contact is persisted.
    scalar distance; // Signed distance along normal.
    std::optional<collision_feature> featureA; // Closest feature on A.
    std::optional<collision_feature> featureB; // Closest feature on B.
};

}

#endif // EDYN_COLLISION_CONTACT_POINT_HPP
