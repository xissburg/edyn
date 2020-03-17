#ifndef EDYN_UTIL_RIGIDBODY_HPP
#define EDYN_UTIL_RIGIDBODY_HPP

#include <optional>
#include <entt/fwd.hpp>
#include "edyn/math/vector3.hpp"
#include "edyn/math/quaternion.hpp"
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
    scalar spin_angle {0};

    // Mass properties for dynamic entities.
    scalar mass {1};
    vector3 inertia {1, 1, 1};

    // Initial linear and angular velocity.
    vector3 linvel {vector3_zero};
    vector3 angvel {vector3_zero};
    scalar spin {0};

    // Gravity acceleration.
    vector3 gravity {gravity_earth};

    // Optional shape for collidable entities.
    std::optional<decltype(shape::var)> shape_opt; 

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

    // Whether this entity will be used for presentation and needs 
    // position/orientation interpolation.
    bool presentation {false};

    void update_inertia();
};

void make_rigidbody(entt::entity, entt::registry &, const rigidbody_def &);
entt::entity make_rigidbody(entt::registry &, const rigidbody_def &);

void update_kinematic_position(entt::registry &, entt::entity, const vector3 &, scalar dt);
void update_kinematic_orientation(entt::registry &, entt::entity, const quaternion &, scalar dt);
void clear_kinematic_velocities(entt::registry &);

}

#endif // EDYN_UTIL_RIGIDBODY_HPP