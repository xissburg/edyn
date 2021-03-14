#ifndef EDYN_MATH_MATRIX3X3_HPP
#define EDYN_MATH_MATRIX3X3_HPP

#include <array>
#include "vector3.hpp"
#include "quaternion.hpp"

namespace edyn {

struct matrix3x3 {
    std::array<vector3, 3> row;

    vector3 & operator[](size_t i) {
        EDYN_ASSERT(i < 3);
        return row[i];
    }

    const vector3 & operator[](size_t i) const {
        EDYN_ASSERT(i < 3);
        return row[i];
    }

    inline vector3 column(size_t i) const {
        return {row[0][i], row[1][i], row[2][i]};
    }

    inline scalar column_dot(size_t i, const vector3 &v) const {
        return row[0][i] * v.x + row[1][i] * v.y + row[2][i] * v.z;
    }
};

// Identity matrix.
inline constexpr matrix3x3 matrix3x3_identity {{vector3_x, vector3_y, vector3_z}};

// Zero matrix.
inline constexpr matrix3x3 matrix3x3_zero {{vector3_zero, vector3_zero, vector3_zero}};

// Add two matrices.
inline matrix3x3 operator+(const matrix3x3 &m, const matrix3x3 &n) {
    return {m.row[0] + n.row[0], m.row[1] + n.row[1], m.row[2] + n.row[2]};
}

// Subtract two matrices.
inline matrix3x3 operator-(const matrix3x3 &m, const matrix3x3 &n) {
    return {m.row[0] - n.row[0], m.row[1] - n.row[1], m.row[2] - n.row[2]};
}

// Multiply two matrices.
inline matrix3x3 operator*(const matrix3x3 &m, const matrix3x3 &n) {
    return {
        vector3{n.column_dot(0, m.row[0]), n.column_dot(1, m.row[0]), n.column_dot(2, m.row[0])},
        vector3{n.column_dot(0, m.row[1]), n.column_dot(1, m.row[1]), n.column_dot(2, m.row[1])},
        vector3{n.column_dot(0, m.row[2]), n.column_dot(1, m.row[2]), n.column_dot(2, m.row[2])}
    };
}

// Multiply vector by matrix.
inline vector3 operator*(const matrix3x3 &m, const vector3 &v) {
    return {dot(m.row[0], v), dot(m.row[1], v), dot(m.row[2], v)};
}

// Multiply vector by matrix on the right, effectively multiplying
// by the transpose.
inline vector3 operator*(const vector3 &v, const matrix3x3 &m) {
    return {m.column_dot(0, v), m.column_dot(1, v), m.column_dot(2, v)};
}

// Transpose of a 3x3 matrix.
inline matrix3x3 transpose(const matrix3x3 &m) {
    return {m.column(0), m.column(1), m.column(2)};
}

// Matrix with given vector as diagonal.
inline matrix3x3 diagonal(const vector3 &v) {
    return {
        vector3 {v.x, 0, 0},
        vector3 {0, v.y, 0},
        vector3 {0, 0, v.z}
    }; 
}

// Equivalent to m * diagonal(v).
inline matrix3x3 scale(const matrix3x3 &m, const vector3 &v) {
    return {
        vector3{m.row[0].x * v.x, m.row[0].y * v.y, m.row[0].z * v.z},
        vector3{m.row[1].x * v.x, m.row[1].y * v.y, m.row[1].z * v.z},
        vector3{m.row[2].x * v.x, m.row[2].y * v.y, m.row[2].z * v.z}
    };
}

// Skew anti-symmetric matrix of a vector.
inline matrix3x3 skew(const vector3 &v) {
    return {
        vector3 {0, -v.z, v.y},
        vector3 {v.z, 0, -v.x},
        vector3 {-v.y, v.x, 0}
    };
}

inline matrix3x3 matrix3x3_columns(const vector3 &v0, 
                                   const vector3 &v1, 
                                   const vector3 &v2) {
    return {
        vector3{v0.x, v1.x, v2.x},
        vector3{v0.y, v1.y, v2.y},
        vector3{v0.z, v1.z, v2.z}
    };
}

inline matrix3x3 to_matrix3x3(const quaternion &q) {
    auto d = length_sqr(q);
    auto s = 2 / d;
    auto xs = q.x * s , ys = q.y * s , zs = q.z * s ;
    auto wx = q.w * xs, wy = q.w * ys, wz = q.w * zs;
    auto xx = q.x * xs, xy = q.x * ys, xz = q.x * zs;
    auto yy = q.y * ys, yz = q.y * zs, zz = q.z * zs;

    return {
        vector3{1 - (yy + zz), xy - wz, xz + wy},
        vector3{xy + wz, 1 - (xx + zz), yz - wx},
        vector3{xz - wy, yz + wx, 1 - (xx + yy)}
    };
}

inline quaternion to_quaternion(const matrix3x3 &m) {
    auto trace = m[0][0] + m[1][1] + m[2][2];

    if (trace > 0) {
        auto s = std::sqrt(trace + scalar(1));
        auto t = scalar(0.5) / s;
        return {t * (m[2][1] - m[1][2]),
                t * (m[0][2] - m[2][0]),
                t * (m[1][0] - m[0][1]),
                scalar(0.5) * s};
    }

    size_t i = m[0][0] < m[1][1] ? (m[1][1] < m[2][2] ? 2 : 1) : (m[0][0] < m[2][2] ? 2 : 0);
    size_t j = (i + 1) % 3;
    size_t k = (i + 2) % 3;
    auto s = std::sqrt(m[i][i] - m[j][j] - m[k][k] + scalar(1));
    auto t = scalar(0.5) / s;
    scalar temp[4];
    temp[i] = scalar(0.5) * s;
    temp[j] = t * (m[j][i] - m[i][j]);
    temp[k] = t * (m[k][i] - m[i][k]);
    temp[3] = t * (m[k][j] - m[j][k]);
    
    return {temp[0], temp[1], temp[2], temp[3]};
}

inline
vector3 to_object_space(const vector3 &p, const vector3 &pos, const matrix3x3 &basis) {
    return transpose(basis) * (p - pos);
}

}

#endif // EDYN_MATH_MATRIX3X3_HPP