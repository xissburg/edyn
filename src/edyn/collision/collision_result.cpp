#include "edyn/collision/collision_result.hpp"
#include "edyn/math/geom.hpp"

namespace edyn {

void collision_result::add_point(const collision_result::collision_point &new_point) {
    EDYN_ASSERT(num_points < max_contacts);
    auto idx = num_points++;
    point[idx] = new_point;
}

void collision_result::maybe_add_point(const collision_result::collision_point &new_point) {
    std::array<vector3, max_contacts> pivots;
    std::array<scalar, max_contacts> distances;
    for (size_t i = 0; i < num_points; ++i) {
        pivots[i] = point[i].pivotA;
        distances[i] = point[i].distance;
    }

    auto idx = insert_index(pivots, distances, num_points, new_point.pivotA, new_point.distance, false);

    // No closest point found for pivotA, try pivotB.
    if (idx >= num_points) {
        for (size_t i = 0; i < num_points; ++i) {
            pivots[i] = point[i].pivotB;
        }

        idx = insert_index(pivots, distances, num_points, new_point.pivotB, new_point.distance, false);
    }

    if (idx < max_contacts) {
        if (idx == num_points) {
            EDYN_ASSERT(num_points < max_contacts);
            ++num_points;
        }
        point[idx] = new_point;
    }
}

}
