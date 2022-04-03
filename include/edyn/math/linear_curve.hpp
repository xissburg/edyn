#ifndef EDYN_MATH_LINEAR_CURVE_HPP
#define EDYN_MATH_LINEAR_CURVE_HPP

#include <cstddef>
#include <vector>
#include "edyn/math/scalar.hpp"

namespace edyn {

/**
 * @brief A piece-wise linear curve,
 */
class linear_curve {
public:
    void add(scalar x, scalar y);
    void clear();
    scalar get(scalar x) const;
    std::pair<scalar, scalar> get(size_t i) const;
    std::size_t size() const { return points.size(); }
    void resize(size_t new_size) { points.resize(new_size); }
    std::pair<scalar, scalar>& operator[](size_t i) { return points[i]; }
    const std::pair<scalar, scalar>& operator[](size_t i) const { return points[i]; }

protected:
    std::vector<std::pair<scalar, scalar>> points;
};

}

#endif // EDYN_MATH_LINEAR_CURVE_HPP
