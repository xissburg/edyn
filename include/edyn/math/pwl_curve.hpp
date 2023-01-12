#ifndef EDYN_MATH_PWL_CURVE_HPP
#define EDYN_MATH_PWL_CURVE_HPP

#include <cstddef>
#include <vector>
#include "edyn/math/scalar.hpp"

namespace edyn {

/**
 * @brief A piece-wise linear curve.
 */
class pwl_curve {
public:
    void add(scalar x, scalar y);
    void clear();
    scalar get(scalar x) const;
    std::pair<scalar, scalar> get(size_t i) const;
    std::pair<scalar, scalar> get_min_max() const;
    std::size_t size() const { return points.size(); }
    void resize(size_t new_size) { points.resize(new_size); }
    void remove(size_t i) { points.erase(points.begin() + i); }
    std::pair<scalar, scalar>& operator[](size_t i) { return points[i]; }
    const std::pair<scalar, scalar>& operator[](size_t i) const { return points[i]; }
    auto begin() const { return points.cbegin(); };
    auto end() const { return points.cend(); };

    template<typename Archive>
    friend void serialize(Archive &, pwl_curve &);

protected:
    std::vector<std::pair<scalar, scalar>> points;
};

template<typename Archive>
void serialize(Archive &archive, pwl_curve &curve) {
    archive(curve.points);
}

}

#endif // EDYN_MATH_PWL_CURVE_HPP
