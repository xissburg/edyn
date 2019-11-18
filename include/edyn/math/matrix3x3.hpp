#ifndef EDYN_MATH_MATRIX3X3_HPP
#define EDYN_MATH_MATRIX3X3_HPP

#include <array>
#include "vector3.hpp"

namespace edyn {

struct matrix3x3 {
    std::array<vector3, 3> row;

    vector3 column(size_t i) const {
        return {row[0][i], row[1][i], row[2][i]};
    }
};

// Identity matrix.
inline constexpr matrix3x3 matrix3x3_identity {{vector3_x, vector3_y, vector3_z}};

// Skew anti-symmetric matrix of a vector.
matrix3x3 skew(const vector3& v) {
    return {vector3 {0, -v.z, v.y},
            vector3 {v.z, 0, -v.x},
            vector3 {-v.y, v.x, 0}};
}

}

#endif // EDYN_MATH_MATRIX3X3_HPP