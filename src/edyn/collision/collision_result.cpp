#include "edyn/collision/collision_result.hpp"

namespace edyn {

void collision_result::add_point(const collision_result::collision_point &new_point) {
    // Ignore if there's already a point that's near this one.
    auto ignore = false;
    for (size_t i = 0; i < num_points; ++i) {
        auto dist_sqr = distance2(new_point.pivotB, point[i].pivotB);
        if (dist_sqr < contact_breaking_threshold * contact_breaking_threshold) {
            ignore = true;
            break;
        }
    }

    if (ignore) {
        return;
    }
    
    if (num_points < max_contacts) {
        auto idx = num_points++;
        point[idx] = new_point;
        return;
    }

    /* auto max_dist = EDYN_SCALAR_MAX;
    size_t max_dist_idx;

    for (size_t i = 0; i < max_contacts; ++i) {
        if (point[i].distance > max_dist) {
            max_dist = point[i].distance;
            max_dist_idx = i;
        }
    }

    if (new_point.distance < max_dist - EDYN_EPSILON) {
        point[max_dist_idx] = new_point;
        return;
    } */


    // Find an existing point to replace. Sort all points anti-clockwise
    // and look for the point that is the closest to being collinear with
    // its neighbors and replace it with the new.
    constexpr auto num_contacts = max_contacts + 1;
    std::array<vector3, num_contacts> points = {
        point[0].pivotB,
        point[1].pivotB,
        point[2].pivotB,
        point[3].pivotB,
        new_point.pivotB,
    };

    // Points will be sorted below. Use an array of indices to refer back
    // to the points in `result.point`.
    std::array<size_t, num_contacts> index_map;
    for (size_t i = 0; i < num_contacts; ++i) {
        index_map[i] = i;
    }

    // Find a second point which connects to the first and has all other
    // points on a single side.
    for (size_t i = 1; i < num_contacts; ++i) {
        auto edge = points[i] - points[0];
        auto tangent = cross(edge, new_point.normalB);
        bool b = true;

        for (size_t j = 1; j < num_contacts; ++j) {
            if (j == i) continue;
            if (dot(points[j] - points[0], tangent) < 0) {
                b = false;
                break;
            }
        }

        if (b) {
            std::swap(points[1], points[i]);
            std::swap(index_map[1], index_map[i]);
            break;
        }
    }

    // Sort points in a winding order (could be CW or CCW, doesn't matter).
    for (size_t i = 1; i < num_contacts - 2; ++i) {
        auto max_dot_edge = -EDYN_SCALAR_MAX;
        auto min_dot_tangent = EDYN_SCALAR_MAX;
        size_t next_idx;
        auto edge = points[i] - points[i - 1];
        auto tangent = cross(edge, new_point.normalB);

        // Find other point that's furthest along edge and closest along tangent.
        for (size_t j = i + 1; j < num_contacts; ++j) {
            auto de = dot(points[j] - points[i - 1], edge);
            if (de > max_dot_edge) {
                auto dt = dot(points[j] - points[i - 1], tangent);

                if (dt < min_dot_tangent) {
                    max_dot_edge = de;
                    min_dot_tangent = dt;
                    next_idx = j;
                }
            }
        }

        std::swap(points[i + 1], points[next_idx]);
        std::swap(index_map[i + 1], index_map[next_idx]);
    }

    // Give each point _reverse_ scores proportional to the angle
    // between the edges connecting it to its immediate neighbors.
    auto scores = make_array<num_contacts>(EDYN_SCALAR_MAX);
    for (size_t i = 0; i < num_contacts; ++i) {
        auto &p0 = points[i];
        auto &p1 = points[(i + (num_contacts - 1)) % num_contacts];
        auto &p2 = points[(i + 1) % num_contacts];
        auto v1 = p1 - p0;
        auto v2 = p2 - p0;
        auto l1 = length2(v1);
        auto l2 = length2(v2);

        if (l1 > EDYN_EPSILON && l2 > EDYN_EPSILON) {
            scores[i] = dot(v1 / l1, v2 / l2);
        }
    }

    // Choose point with lowest score.
    auto min_score = EDYN_SCALAR_MAX;
    size_t min_score_idx;
    for (size_t i = 0; i < num_contacts; ++i) {
        if (scores[i] < min_score) {
            min_score = scores[i];
            min_score_idx = i;
        }
    }

    // If the point with lowest score is not the new point, replace it
    // by the new point.
    if (index_map[min_score_idx] < max_contacts) {
        auto idx = index_map[min_score_idx];
        point[idx] = new_point;
    }
}

}