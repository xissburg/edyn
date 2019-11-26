#ifndef EDYN_COLLISION_COLLISION_RESULT_HPP
#define EDYN_COLLISION_COLLISION_RESULT_HPP

#include <array>
#include "edyn/math/vector3.hpp"
#include "edyn/math/constants.hpp"

namespace edyn {

struct collision_result {
    struct collision_point {
        vector3 pivotA;
        vector3 pivotB;
        vector3 normalB;
    };

    size_t num_points {0};
    std::array<collision_point, max_contacts> point;
};

}

#endif // EDYN_COLLISION_COLLISION_RESULT_HPP