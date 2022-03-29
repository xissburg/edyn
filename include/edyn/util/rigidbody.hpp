#ifndef EDYN_UTIL_RIGIDBODY_HPP
#define EDYN_UTIL_RIGIDBODY_HPP

#include <vector>
#include <optional>
#include <entt/entity/fwd.hpp>
#include "edyn/math/vector3.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/shapes/shapes.hpp"
#include "edyn/comp/material.hpp"
#include "edyn/comp/collision_filter.hpp"

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

    // Center of mass offset from origin in object space.
    std::optional<vector3> center_of_mass;

    // Gravity acceleration. If not set, the default value from
    // `edyn::get_gravity` will be assigned.
    std::optional<vector3> gravity;

    // Optional shape for collidable entities.
    std::optional<shapes_variant_t> shape;

    // Optional material. If not set, the rigid body will not respond to
    // collisions, i.e. it becomes a _sensor_.
    std::optional<edyn::material> material {edyn::material{}};

    uint64_t collision_group {collision_filter::all_groups};
    uint64_t collision_mask {collision_filter::all_groups};

    // Mark all contacts involving this rigid body as continuous.
    bool continuous_contacts {false};

    // Whether this entity will be used for presentation and needs
    // position/orientation interpolation.
    bool presentation {true};

    // Prevent this rigid body from sleeping while it barely moves.
    bool sleeping_disabled {false};

    // Share this rigid body over the network.
    bool networked {false};

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
/**
 * @brief Applies a rotational impulse to entity.
 * @param registry Data source.
 * @param entity Rigid body entity.
 * @param torque_impulse Torque impulse vector.
 */
void rigidbody_apply_torque_impulse(entt::registry &, entt::entity,
                                    const vector3 &torque_impulse);

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

/**
 * @brief Offset the rigid body center of mass. The value represents an offset
 * from the rigid body's origin in object space.
 * @remark When the center of mass offset changes, the position and linear
 * velocity change as well to reflect the value in the new location. This
 * dependency makes it necessary to do these updates in the island worker or
 * else the simulation will be slightly disturbed. For this reason, this call
 * does not immediately update the center of mass.
 * @param registry Data source.
 * @param entity Rigid body entity.
 * @param com Center of mass offset.
 */
void set_center_of_mass(entt::registry &, entt::entity, const vector3 &com);

void apply_center_of_mass(entt::registry &, entt::entity, const vector3 &com);

/**
 * @brief Get location of rigid body's origin in world space. The position and
 * origin will match if the center of mass offset is zero.
 * @param registry Data source.
 * @param entity Rigid body entity.
 * @return Origin location in the world.
 */
vector3 get_rigidbody_origin(const entt::registry &, entt::entity);

/**
 * @brief Get interpolated location of rigid body's origin in world space for
 * presentation.
 * @param registry Data source.
 * @param entity Rigid body entity.
 * @return Origin location in the world for presentation.
 */
vector3 get_rigidbody_present_origin(const entt::registry &, entt::entity);

}

#endif // EDYN_UTIL_RIGIDBODY_HPP
