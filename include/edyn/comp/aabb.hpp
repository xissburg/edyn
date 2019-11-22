#ifndef EDYN_COMP_AABB_HPP
#define EDYN_COMP_AABB_HPP

#include "edyn/math/vector3.hpp"

namespace edyn {

struct AABB {
    vector3 min;
    vector3 max;
};

}

#endif // EDYN_COMP_AABB_HPP