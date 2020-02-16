#ifndef EDYN_UTIL_TIRE_HPP
#define EDYN_UTIL_TIRE_HPP

#include <cstdint>
#include "edyn/math/scalar.hpp"

namespace edyn {

struct tire_state {
    scalar vertical_load {0};
    scalar vertical_deflection {0};
    scalar speed {0};
    scalar inflation_pressure {200000};
    scalar friction_coefficient {1};
    scalar sin_camber {0};
    scalar slip_angle {0};
    scalar slip_ratio {0};
    scalar yaw_rate {0};
    scalar Fx {0}, Fy {0}, Mz {0}, Mx {0};
    scalar slide_factor {0};
    scalar contact_patch_length {0};
    scalar contact_patch_width {0};
};

struct tire_specification {
    uint8_t num_bristles {10};
    uint8_t num_rows {3};
    scalar radius {0.34};
    scalar width {0.204};
    scalar tread_width {0.192};
    scalar vertical_stiffness {100000};
    scalar vertical_damping {1500};
    scalar friction_coefficient {1.2};
    scalar longitudinal_tread_stiffness {3600000};
    scalar lateral_tread_stiffness {2000000};
    scalar friction_velocity_sensitivity {0.03};
    scalar load_sensitivity {0.05};
    scalar carcass_torsional_stiffness {2000};
    scalar carcass_torsional_damping {10};
    scalar carcass_longitudinal_stiffness {4000};
    scalar carcass_longitudinal_damping {30};
    scalar carcass_lateral_stiffness {6000};
    scalar carcass_lateral_damping {40};
};

void calculate_tire_forces(const tire_specification &, tire_state &);
scalar velocity_dependent_vertical_stiffness(scalar nominal_stiffness, 
                                             scalar speed, 
                                             scalar inflation_pressure = 200000);

}

#endif // EDYN_UTIL_TIRE_HPP