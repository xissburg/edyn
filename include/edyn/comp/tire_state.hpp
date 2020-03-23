#ifndef EDYN_COMP_TIRE_STATE_HPP
#define EDYN_COMP_TIRE_STATE_HPP

#include <cstdint>
#include <entt/fwd.hpp>
#include "edyn/math/vector3.hpp"

namespace edyn {

struct tire_state {
    scalar vertical_deflection {0};
    scalar speed {0};
    scalar inflation_pressure {200000};
    scalar friction_coefficient {1};
    scalar sin_camber {0};
    scalar slip_angle {0};
    scalar slip_ratio {0};
    scalar yaw_rate {0};
    scalar Fx {0}, Fy {0}, Fz {0}, Mz {0}, Mx {0};
    scalar slide_factor {0};
    scalar contact_patch_length {0};
    scalar contact_patch_width {0};
    uint32_t contact_lifetime {0};
    bool in_contact;
    entt::entity other_entity;
    vector3 lon_dir;
    vector3 lat_dir;
    vector3 normal;
    vector3 position;
    vector3 lin_vel;
};

}


#endif // EDYN_COMP_TIRE_STATE_HPP