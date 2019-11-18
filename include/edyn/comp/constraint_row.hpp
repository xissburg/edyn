#ifndef EDYN_COMP_CONSTRAINT_ROW_HPP
#define EDYN_COMP_CONSTRAINT_ROW_HPP

#include <array>
#include "edyn/math/vector3.hpp"

namespace edyn {

struct constraint_row {
    std::array<vector3, 4> J;
    scalar error;
    scalar lower_limit;
    scalar upper_limit;
};

}

#endif // EDYN_COMP_CONSTRAINT_ROW_HPP