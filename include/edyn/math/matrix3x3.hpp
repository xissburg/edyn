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

// Zero matrix.
inline constexpr matrix3x3 matrix3x3_zero {{vector3_zero, vector3_zero, vector3_zero}};

// Multiply vector by matrix.
inline vector3 operator*(const matrix3x3& m, const vector3& v) {
    return {dot(m.row[0], v), dot(m.row[1], v), dot(m.row[2], v)};
}

// Matrix with given vector as diagonal.
inline matrix3x3 diagonal(const vector3& v) {
    return {vector3 {v.x, 0, 0},
            vector3 {0, v.y, 0},
            vector3 {0, 0, v.z}}; 
}

// Skew anti-symmetric matrix of a vector.
inline matrix3x3 skew(const vector3& v) {
    return {vector3 {0, -v.z, v.y},
            vector3 {v.z, 0, -v.x},
            vector3 {-v.y, v.x, 0}};
}

}

#endif // EDYN_MATH_MATRIX3X3_HPP