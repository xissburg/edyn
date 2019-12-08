#ifndef EDYN_COLLISION_COLLISION_RESULT_HPP
#define EDYN_COLLISION_COLLISION_RESULT_HPP

#include <array>
#include <utility>
#include "edyn/math/vector3.hpp"
#include "edyn/math/constants.hpp"

namespace edyn {

struct collision_result {
    struct collision_point {
        vector3 pivotA;
        vector3 pivotB;
        vector3 normalB;
        scalar distance;
    };

    size_t num_points {0};
    std::array<collision_point, max_contacts> point;

    collision_result & swap() {
        for (size_t i = 0; i < num_points; ++i) {
            auto &cp = point[i];
            std::swap(cp.pivotA, cp.pivotB);
            cp.normalB = -cp.normalB;
        }
        return *this;
    }
};

}

#endif // EDYN_COLLISION_COLLISION_RESULT_HPP