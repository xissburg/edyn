#ifndef EDYN_COMP_CON_ROW_ITER_DATA
#define EDYN_COMP_CON_ROW_ITER_DATA

#include "edyn/math/matrix3x3.hpp"

namespace edyn {

struct delta_linvel;
struct delta_angvel;

struct con_row_iter_data {
    std::array<vector3, 2 * 2> J;
    delta_linvel *dvA, *dvB;
    delta_angvel *dwA, *dwB;
    scalar rhs;
    scalar eff_mass;
    scalar lower_limit;
    scalar upper_limit;
    scalar impulse;
    scalar inv_mA, inv_mB;
    matrix3x3 inv_IA, inv_IB;
};

}

#endif // EDYN_COMP_CON_ROW_ITER_DATA