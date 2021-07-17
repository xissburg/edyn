#ifndef EDYN_COLLISION_COLLISION_RESULT_HPP
#define EDYN_COLLISION_COLLISION_RESULT_HPP

#include <array>
#include <utility>
#include "edyn/math/quaternion.hpp"
#include "edyn/config/constants.hpp"
#include "edyn/util/array.hpp"

namespace edyn {

struct collision_result {
    struct collision_point {
        vector3 pivotA;
        vector3 pivotB;
        vector3 normal;
        scalar distance;
    };

    size_t num_points {0};
    std::array<collision_point, max_contacts> point;

    collision_result & swap(const quaternion &ornA, const quaternion &ornB) {
        for (size_t i = 0; i < num_points; ++i) {
            auto &cp = point[i];
            std::swap(cp.pivotA, cp.pivotB);
            cp.normal *= -1; // Point towards new A.
        }
        return *this;
    }

    void add_point(const collision_point &);
    void maybe_add_point(const collision_point &);
};

}

#endif // EDYN_COLLISION_COLLISION_RESULT_HPP