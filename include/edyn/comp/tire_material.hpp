#ifndef EDYN_COMP_TIRE_MATERIAL_HPP
#define EDYN_COMP_TIRE_MATERIAL_HPP

#include "edyn/math/scalar.hpp"

namespace edyn {

struct tire_material {
    scalar vertical_stiffness {250000};
    scalar vertical_damping {1200};
    scalar lon_tread_stiffness {10e6};
    scalar lat_tread_stiffness {6e6};
    scalar max_tread_deflection{0.03};
    scalar speed_sensitivity {0.03};
    scalar load_sensitivity {0.05};
    scalar tire_radius {0.36};
    scalar rim_radius {0.19};
    scalar tread_width {0.21};
    scalar nominal_inflation_pressure {30};
    scalar vertical_stiffness_inflation_pressure_rate {5000};
};

template<typename Archive>
void serialize(Archive &archive, tire_material &mat) {
    archive(mat.vertical_stiffness, mat.vertical_damping);
    archive(mat.lon_tread_stiffness, mat.lat_tread_stiffness);
    archive(mat.max_tread_deflection);
    archive(mat.speed_sensitivity, mat.load_sensitivity);
    archive(mat.tire_radius, mat.rim_radius, mat.tread_width);
    archive(mat.nominal_inflation_pressure);
    archive(mat.vertical_stiffness_inflation_pressure_rate);
}

}

#endif // EDYN_COMP_TIRE_MATERIAL_HPP
