#ifndef EDYN_COMP_TIRE_STATE_HPP
#define EDYN_COMP_TIRE_STATE_HPP

#include <array>
#include <entt/entity/entity.hpp>
#include <vector>
#include <cstdint>
#include <entt/fwd.hpp>
#include "edyn/math/vector3.hpp"
#include "edyn/config/constants.hpp"
#include "edyn/constraints/contact_patch_constraint.hpp"

namespace edyn {

struct tire_bristle_state {
    vector3 root;
    vector3 tip;
    scalar friction;
    scalar sliding_spd;
};

struct tire_tread_row_state {
    vector3 start_pos;
    vector3 end_pos;
    std::array<tire_bristle_state, contact_patch_constraint::bristles_per_row> bristles;
};

struct tire_contact_state {
    // Vertical deflection.
    scalar vertical_deflection {0};

    // Current friction coefficient.
    scalar friction_coefficient {1};

    // Sine of camber angle.
    scalar sin_camber {0};

    // Slip angle.
    scalar slip_angle {0};

    // Slip ratio.
    scalar slip_ratio {0};

    // Angular velocity along contact normal.
    scalar yaw_rate {0};

    // Longitudinal, lateral and normal forces.
    scalar Fx {0}, Fy {0}, Fz {0};

    // Self-aligning moment and rolling moment.
    scalar Mz {0}, Mx {0};

    // Average sliding speed of all bristles.
    scalar slide_speed {0};

    // Percentage of treads which are sliding in [0, 1].
    scalar slide_ratio {0};

    // Width of contact patch.
    scalar contact_patch_width {0};

    // Lifetime of contact point.
    uint32_t contact_lifetime {0};

    // Longitudinal direction.
    vector3 lon_dir;

    // Lateral direction.
    vector3 lat_dir;

    // Contact normal.
    vector3 normal;

    // Pivot point where forces are applied in world space.
    vector3 pivot;

    // Center of contact patch in world space.
    vector3 position;

    // Linear relative velocity.
    vector3 lin_vel;

    std::array<tire_tread_row_state, contact_patch_constraint::num_tread_rows> tread_rows;
};

struct tire_state {
    entt::entity other_entity {entt::null};
    entt::entity patch_entity {entt::null};
    scalar inflation_pressure {200000};
    size_t num_contacts;
    std::array<tire_contact_state, max_contacts> contact_state;
};

}


#endif // EDYN_COMP_TIRE_STATE_HPP
