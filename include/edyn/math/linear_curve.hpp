#ifndef EDYN_MATH_LINEAR_CURVE_HPP
#define EDYN_MATH_LINEAR_CURVE_HPP

#include <vector>
#include "edyn/math/scalar.hpp"

namespace edyn {

class linear_curve {
public:
    void add(scalar x, scalar y);
    void clear();
    scalar get(scalar x) const;
    std::size_t size() const { return points.size(); }

    template<typename Archive>
    friend void Serialize(Archive&, linear_curve&);

protected:
    std::vector<std::pair<scalar, scalar>> points;
};

}

#endif // EDYN_MATH_LINEAR_CURVE_HPP