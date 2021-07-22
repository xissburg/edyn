#ifndef EDYN_UTIL_RIGIDBODY_HPP
#define EDYN_UTIL_RIGIDBODY_HPP

#include <vector>
#include <optional>
#include <entt/entity/fwd.hpp>
#include "edyn/math/vector3.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/shapes/shapes.hpp"

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
    scalar spin_angle {0};

    // Mass properties for dynamic entities.
    scalar mass {1};
    matrix3x3 inertia {matrix3x3_identity};

    // Initial linear and angular velocity.
    vector3 linvel {vector3_zero};
    vector3 angvel {vector3_zero};
    scalar spin {0};

    // Gravity acceleration. If not set, the default value from
    // `edyn::get_gravity` will be assigned.
    std::optional<vector3> gravity;

    // Optional shape for collidable entities.
    std::optional<shapes_variant_t> shape;

    scalar restitution {0};
    scalar friction {0.5};
    scalar stiffness {large_scalar};
    scalar damping {large_scalar};

    bool sensor {false};

    bool spin_enabled {false};

    bool is_tire {false};
    scalar lon_tread_stiffness {3000000};
    scalar lat_tread_stiffness {1800000};
    scalar speed_sensitivity {0.03};
    scalar load_sensitivity {0.05};

    uint64_t collision_group {~0ULL};
    uint64_t collision_mask {~0ULL};

    // Mark all contacts involving this rigid body as continuous.
    bool continuous_contacts {false};

    // Whether this entity will be used for presentation and needs
    // position/orientation interpolation.
    bool presentation {true};

    /**
     * @brief Assigns the default moment of inertia of the current shape
     * using the current mass.
     * Assumes `shape` to contain a value.
     */
    void update_inertia();
};

/**
 * @brief Assigns to `entity` all necessary components to build a rigid body
 * according to the given definition.
 * @param entity Target rigid body entity.
 * @param registry Data source and destination.
 * @param def Rigid body definition.
 */
void make_rigidbody(entt::entity, entt::registry &, const rigidbody_def &);

/**
 * @brief Creates an entity and assigns all necessary components to build a
 * rigid body according to the given definition.
 * @param registry Data source and destination.
 * @param def Rigid body definition.
 * @return Rigid body entity.
 */
entt::entity make_rigidbody(entt::registry &, const rigidbody_def &);

/**
 * @brief Creates many rigid bodies at once.
 * A new island is created and all bodies are inserted into it.
 * @param registry Data source.
 * @param defs Rigid body definitions.
 * @return Entities corresponding to each rigid body definition.
 */
std::vector<entt::entity> batch_rigidbodies(entt::registry &registry, const std::vector<rigidbody_def> &defs);

/**
 * @brief Applies `impulse` to entity.
 * @param registry Data source.
 * @param entity Rigid body entity.
 * @param impulse Impulse vector.
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

/**
 * @brief Set the mass of a rigid body.
 * Propagates changes across island workers.
 * @param registry Data source.
 * @param entity Rigid body entity.
 * @param mass The new rigid body mass.
 */
void set_rigidbody_mass(entt::registry &, entt::entity, scalar mass);

/**
 * @brief Set the moment of inertia of a rigid body.
 * Propagates changes across island workers.
 * @param registry Data source.
 * @param entity Rigid body entity.
 * @param inertia The new moment of inertia.
 */
void set_rigidbody_inertia(entt::registry &, entt::entity, const matrix3x3 &inertia);

/**
 * @brief Besides assigning the new friction coefficient to the rigid body's
 * material, it also updates the friction of the contact points of all
 * manifolds involving this rigid body.
 * @param registry Data source.
 * @param entity Rigid body entity.
 * @param friction The new friction coefficient.
 */
void set_rigidbody_friction(entt::registry &, entt::entity, scalar);

}

#endif // EDYN_UTIL_RIGIDBODY_HPP
