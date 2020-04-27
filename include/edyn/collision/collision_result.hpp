#ifndef EDYN_COLLISION_COLLISION_RESULT_HPP
#define EDYN_COLLISION_COLLISION_RESULT_HPP

#include <array>
#include <utility>
#include "edyn/math/quaternion.hpp"
#include "edyn/math/constants.hpp"
#include "edyn/util/array.hpp"

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

    collision_result & swap(const quaternion &ornA, const quaternion &ornB) {
        for (size_t i = 0; i < num_points; ++i) {
            auto &cp = point[i];
            std::swap(cp.pivotA, cp.pivotB);
            cp.normalB = rotate(conjugate(ornA), -rotate(ornB, cp.normalB));
        }
        return *this;
    }

    void add_point(const collision_point &);
    void maybe_add_point(const collision_point &);
};

}

#endif // EDYN_COLLISION_COLLISION_RESULT_HPP