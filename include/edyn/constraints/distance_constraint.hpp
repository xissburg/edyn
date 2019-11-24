#ifndef EDYN_CONSTRAINTS_DISTANCE_CONSTRAINT_HPP
#define EDYN_CONSTRAINTS_DISTANCE_CONSTRAINT_HPP

#include <array>
#include "edyn/math/vector3.hpp"

namespace edyn {

struct distance_constraint {
    std::array<vector3, 2> pivot;
    scalar distance {0};

    
};

}

#endif // EDYN_CONSTRAINTS_DISTANCE_CONSTRAINT_HPP