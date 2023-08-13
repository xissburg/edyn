#ifndef EDYN_COMP_TIRE_MATERIAL_HPP
#define EDYN_COMP_TIRE_MATERIAL_HPP

#include "edyn/math/scalar.hpp"

namespace edyn {

struct tire_material {
    scalar lon_tread_stiffness {3000000};
    scalar lat_tread_stiffness {1800000};
    scalar tread_damping {1000};
    scalar max_tread_deflection{0.03};
    scalar speed_sensitivity {0.03};
    scalar load_sensitivity {0.05};
    scalar tire_radius {0.36};
    scalar rim_radius {0.19};
};

template<typename Archive>
void serialize(Archive &archive, tire_material &mat) {
    archive(mat.lon_tread_stiffness);
    archive(mat.lat_tread_stiffness);
    archive(mat.tread_damping);
    archive(mat.max_tread_deflection);
    archive(mat.speed_sensitivity);
    archive(mat.load_sensitivity);
    archive(mat.tire_radius, mat.rim_radius);
}

}

#endif // EDYN_COMP_TIRE_MATERIAL_HPP
