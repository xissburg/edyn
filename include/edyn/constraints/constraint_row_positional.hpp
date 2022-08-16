#ifndef EDYN_CONSTRAINT_CONSTRAINT_ROW_POSITIONAL_HPP
#define EDYN_CONSTRAINT_CONSTRAINT_ROW_POSITIONAL_HPP

#include "edyn/config/constants.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/comp/inertia.hpp"

namespace edyn {

struct constraint_row_positional {
    std::array<vector3, 2 * max_constrained_entities> J;
    scalar error;
    scalar inv_mA, inv_mB;
    inertia_world_inv *inv_IA, *inv_IB;
    position *posA, *posB;
    orientation *ornA, *ornB;
};

}

#endif // EDYN_CONSTRAINT_CONSTRAINT_ROW_POSITIONAL_HPP
