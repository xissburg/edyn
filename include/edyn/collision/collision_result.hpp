#ifndef EDYN_COLLISION_COLLISION_RESULT_HPP
#define EDYN_COLLISION_COLLISION_RESULT_HPP

#include <array>
#include <utility>
#include <optional>
#include "edyn/config/constants.hpp"
#include "edyn/collision/contact_normal_attachment.hpp"
#include "edyn/collision/collision_feature.hpp"

namespace edyn {

struct collision_result {
    struct collision_point {
        vector3 pivotA;
        vector3 pivotB;
        vector3 normal;
        scalar distance;
        contact_normal_attachment normal_attachment;
        std::optional<collision_feature> featureA;
        std::optional<collision_feature> featureB;

        collision_point & swap() {
            std::swap(pivotA, pivotB);
            std::swap(featureA, featureB);
            normal *= -1; // Point towards new A.

            if (normal_attachment == contact_normal_attachment::normal_on_A) {
                normal_attachment = contact_normal_attachment::normal_on_B;
            } else if (normal_attachment == contact_normal_attachment::normal_on_B) {
                normal_attachment = contact_normal_attachment::normal_on_A;
            }

            return *this;
        }
    };

    size_t num_points {0};
    std::array<collision_point, max_contacts> point;

    collision_result & swap() {
        for (size_t i = 0; i < num_points; ++i) {
            point[i].swap();
        }
        return *this;
    }

    void add_point(const collision_point &);
    void maybe_add_point(const collision_point &);
};

}

#endif // EDYN_COLLISION_COLLISION_RESULT_HPP
