#ifndef EDYN_COMP_CONSTRAINT_ROW_ITER_DATA
#define EDYN_COMP_CONSTRAINT_ROW_ITER_DATA

#include "edyn/math/matrix3x3.hpp"

namespace edyn {

struct delta_linvel;
struct delta_angvel;

struct constraint_row_iter_data {
    scalar mA, mB;
    matrix3x3 inv_IA, inv_IB;
    delta_linvel *dvA, *dvB;
    delta_angvel *dwA, *dwB;
};

}

#endif // EDYN_COMP_CONSTRAINT_ROW_ITER_DATA