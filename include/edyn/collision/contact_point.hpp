#ifndef EDYN_COLLISION_CONTACT_POINT_HPP
#define EDYN_COLLISION_CONTACT_POINT_HPP

#include <array>
#include <cstdint>
#include <optional>
#include <entt/entity/fwd.hpp>
#include <entt/entity/entity.hpp>
#include "edyn/config/constants.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/collision/contact_normal_attachment.hpp"
#include "edyn/collision/collision_feature.hpp"

namespace edyn {

struct contact_point {
    vector3 pivotA; // A's pivot in object space.
    vector3 pivotB; // B's pivot in object space.
    vector3 normal; // Normal in world space.
    uint32_t lifetime {0}; // Incremented in each simulation step where the contact is persisted.
};

struct contact_point_list {
    entt::entity parent {entt::null};
    entt::entity next {entt::null};
};

struct contact_point_geometry {
    vector3 local_normal; // Normal in object space.
    contact_normal_attachment normal_attachment; // To which body the normal is attached.
    scalar distance; // Signed distance along normal.
    std::optional<collision_feature> featureA; // Closest feature on A.
    std::optional<collision_feature> featureB; // Closest feature on B.
};

struct contact_point_material {
    scalar friction; // Combined friction coefficient.
    scalar spin_friction; // Combined spin friction coefficient.
    scalar roll_friction; // Combined rolling friction coefficient.
    scalar restitution; // Combined coefficient of restitution.
    scalar stiffness {large_scalar};
    scalar damping {large_scalar};
};

struct contact_point_impulse {
    scalar normal_impulse {scalar(0)}; // Applied normal impulse.
    std::array<scalar, 2> friction_impulse {scalar(0), scalar(0)}; // Applied tangential friction impulse.
    scalar normal_restitution_impulse {scalar(0)}; // Applied normal impulse in restitution solver.
    std::array<scalar, 2> friction_restitution_impulse {scalar(0), scalar(0)}; // Applied tangential friction impulse in restitution solver.
    /**
     * The restitution impulses are calculated by the restitution solver and are
     * kept separate because if mixed with `normal_impulse` and `friction_impulse`,
     * the constraint solver will remove some of the propagated shock and the
     * results will not be correct. Add them up to get the full normal and
     * friction impulses.
     */
};

struct contact_point_spin_friction_impulse {
    scalar spin_friction_impulse {scalar(0)}; // Applied spin friction impulse.
};

struct contact_point_roll_friction_impulse {
    std::array<scalar, 2> rolling_friction_impulse {scalar(0), scalar(0)}; // Applied rolling friction impulse.
};

template<typename Archive>
void serialize(Archive &archive, contact_point &cp) {
    archive(cp.pivotA, cp.pivotB);
    archive(cp.normal);
    archive(cp.lifetime);
}

template<typename Archive>
void serialize(Archive &archive, contact_point_list &cp) {
    archive(cp.parent, cp.next);
}

template<typename Archive>
void serialize(Archive &archive, contact_point_geometry &cp) {
    archive(cp.local_normal);
    archive(cp.normal_attachment);
    archive(cp.distance);
    archive(cp.featureA, cp.featureB);
}

template<typename Archive>
void serialize(Archive &archive, contact_point_material &cp) {
    archive(cp.friction);
    archive(cp.spin_friction);
    archive(cp.roll_friction);
    archive(cp.restitution);
    archive(cp.stiffness);
    archive(cp.damping);
}

template<typename Archive>
void serialize(Archive &archive, contact_point_impulse &cp) {
    archive(cp.normal_impulse);
    archive(cp.friction_impulse);
    archive(cp.normal_restitution_impulse);
    archive(cp.friction_restitution_impulse);
}

template<typename Archive>
void serialize(Archive &archive, contact_point_spin_friction_impulse &cp) {
    archive(cp.spin_friction_impulse);
}

template<typename Archive>
void serialize(Archive &archive, contact_point_roll_friction_impulse &cp) {
    archive(cp.rolling_friction_impulse);
}

}

#endif // EDYN_COLLISION_CONTACT_POINT_HPP
