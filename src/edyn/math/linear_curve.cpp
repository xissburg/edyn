#include "edyn/math/linear_curve.hpp"
#include "edyn/config/config.h"

namespace edyn {

void linear_curve::add(scalar x, scalar y) {
    points.push_back(std::pair(x, y));
}

void linear_curve::clear() {
    points.clear();
}

scalar linear_curve::get(scalar x) const {
    EDYN_ASSERT(!points.empty());

    size_t low = 0;
    size_t high = points.size() - 1;

    while (high - low > 1) {
        size_t middle = (low + high) / 2;

        if (x > points[middle].first) {
            low = middle;
        } else {
            high = middle;
        }
    }

    auto x0 = points[low].first;
    auto x1 = points[high].first;
    auto y0 = points[low].second;
    auto y1 = points[high].second;

    auto y = y0 + (y1 - y0) * ((x - x0) / (x1 - x0));

    return y;
}

std::pair<scalar, scalar> linear_curve::get(size_t i) const {
    return points[i];
}

}