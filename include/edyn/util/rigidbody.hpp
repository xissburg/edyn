#ifndef EDYN_UTIL_RIGIDBODY_HPP
#define EDYN_UTIL_RIGIDBODY_HPP

#include <optional>
#include <entt/fwd.hpp>
#include "edyn/math/vector3.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/comp/shape.hpp"

namespace edyn {

enum class rigidbody_kind : uint8_t {
    // A rigid body with non-zero and finite mass that reacts to forces and
    // impulses and can be affected by constraints.
    rb_dynamic,

    // A rigid body that is not affected by others and can be moved directly.
    rb_kinematic,

    // A rigid body that is not affected by others and never changes.
    rb_static
};

struct rigidbody_def {
    // The entity kind will determine which components are added to it
    // in the `make_rigidbody` call.
    rigidbody_kind kind {rigidbody_kind::rb_dynamic};

    // Initial position and orientation.
    vector3 position {vector3_zero};
    quaternion orientation {quaternion_identity};

    // Mass properties for dynamic entities.
    scalar mass {1};
    matrix3x3 inertia {matrix3x3_identity};

    // Initial linear and angular velocity.
    vector3 linvel {vector3_zero};
    vector3 angvel {vector3_zero};

    // Gravity acceleration.
    vector3 gravity {gravity_earth};

    // Optional shape for collidable entities.
    std::optional<shape> shape_opt; 

    scalar restitution {0};
    scalar friction {0.5};
    scalar stiffness {large_scalar};
    scalar damping {large_scalar};

    bool sensor {false};

    uint64_t collision_group {1ULL};
    uint64_t collision_mask {~0ULL};

    // Mark all contacts involving this rigid body as continuous.
    bool continuous_contacts {false};

    // Whether this entity will be used for presentation and needs 
    // position/orientation interpolation.
    bool presentation {false};

    void update_inertia();
};

/**
 * Assigns to `entity` all necessary components to build a rigid body according
 * to the given definition.
 */
void make_rigidbody(entt::entity, entt::registry &, const rigidbody_def &);
entt::entity make_rigidbody(entt::registry &, const rigidbody_def &);

/**
 * Sets the mass of a rigid body and recalculates its inertia. It assumes the
 * entity has a shape associated to it, thus it must not be used with implicit
 * rigid bodies.
 */
void rigidbody_set_mass(entt::registry &, entt::entity, scalar mass);

/**
 * Recalculates the inertia of a rigid body. Must be called after the shape of
 * a body changes. It assumes the entity has a shape associated to it, thus it
 * must not be used with implicit rigid bodies.
 */
void rigidbody_update_inertia(entt::registry &, entt::entity);

/**
 * Applies `impulse` to entity.
 * @param rel_location Location where the impulse should be applied relative to
 * the entity's center/position, in world space, i.e.
 * `actual_world_space_location - position`.
 */
void rigidbody_apply_impulse(entt::registry &, entt::entity, 
                             const vector3 &impulse, 
                             const vector3 &rel_location);

void update_kinematic_position(entt::registry &, entt::entity, const vector3 &, scalar dt);
void update_kinematic_orientation(entt::registry &, entt::entity, const quaternion &, scalar dt);
void clear_kinematic_velocities(entt::registry &);

bool validate_rigidbody(entt::entity &, entt::registry &);
}

#endif // EDYN_UTIL_RIGIDBODY_HPP