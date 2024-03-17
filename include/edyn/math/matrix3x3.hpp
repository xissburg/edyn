#ifndef EDYN_MATH_MATRIX3X3_HPP
#define EDYN_MATH_MATRIX3X3_HPP

#include <array>
#include "edyn/config/config.h"
#include "edyn/math/constants.hpp"
#include "edyn/math/vector3.hpp"
#include "edyn/math/quaternion.hpp"

namespace edyn {

struct matrix3x3 {
    std::array<vector3, 3> row;

    constexpr vector3& operator[](size_t i) noexcept {
        EDYN_ASSERT(i < 3);
        return row[i];
    }

    constexpr const vector3& operator[](size_t i) const noexcept {
        EDYN_ASSERT(i < 3);
        return row[i];
    }

    constexpr vector3 column(size_t i) const noexcept {
        return {row[0][i], row[1][i], row[2][i]};
    }

    constexpr scalar column_dot(size_t i, const vector3 &v) const noexcept {
        return row[0][i] * v.x + row[1][i] * v.y + row[2][i] * v.z;
    }

    constexpr scalar determinant() const noexcept {
        return triple_product(row[0], row[1], row[2]);
    }
};

// Identity matrix.
inline constexpr matrix3x3 matrix3x3_identity {{vector3_x, vector3_y, vector3_z}};

// Zero matrix.
inline constexpr matrix3x3 matrix3x3_zero {{vector3_zero, vector3_zero, vector3_zero}};

// Matrix with all ones.
inline constexpr matrix3x3 matrix3x3_one {{vector3_one, vector3_one, vector3_one}};

// Add two matrices.
constexpr matrix3x3 operator+(const matrix3x3 &m, const matrix3x3 &n) noexcept {
    return {m.row[0] + n.row[0], m.row[1] + n.row[1], m.row[2] + n.row[2]};
}

// Subtract two matrices.
constexpr matrix3x3 operator-(const matrix3x3 &m, const matrix3x3 &n) noexcept {
    return {m.row[0] - n.row[0], m.row[1] - n.row[1], m.row[2] - n.row[2]};
}

// Multiply two matrices.
constexpr matrix3x3 operator*(const matrix3x3 &m, const matrix3x3 &n) noexcept {
    return {
        vector3{n.column_dot(0, m.row[0]), n.column_dot(1, m.row[0]), n.column_dot(2, m.row[0])},
        vector3{n.column_dot(0, m.row[1]), n.column_dot(1, m.row[1]), n.column_dot(2, m.row[1])},
        vector3{n.column_dot(0, m.row[2]), n.column_dot(1, m.row[2]), n.column_dot(2, m.row[2])}
    };
}

// Multiply vector by matrix.
constexpr vector3 operator*(const matrix3x3 &m, const vector3 &v) noexcept {
    return {dot(m.row[0], v), dot(m.row[1], v), dot(m.row[2], v)};
}

// Multiply vector by matrix on the right, effectively multiplying
// by the transpose.
constexpr vector3 operator*(const vector3 &v, const matrix3x3 &m) noexcept {
    return {m.column_dot(0, v), m.column_dot(1, v), m.column_dot(2, v)};
}

// Multiply matrix by scalar.
constexpr matrix3x3 operator*(const matrix3x3& m, scalar s) noexcept {
    return {m[0] * s, m[1] * s, m[2] * s};
}

// Multiply scalar by matrix.
constexpr matrix3x3 operator*(scalar s, const matrix3x3& m) noexcept {
    return {s * m[0], s * m[1], s * m[2]};
}

// Add one matrix to another.
constexpr matrix3x3 & operator+=(matrix3x3 &m, const matrix3x3 &n) noexcept {
    m.row[0] += n.row[0];
    m.row[1] += n.row[1];
    m.row[2] += n.row[2];
    return m;
}

// Subtract one matrix from another.
constexpr matrix3x3 operator-=(matrix3x3 &m, const matrix3x3 &n) noexcept {
    m.row[0] -= n.row[0];
    m.row[1] -= n.row[1];
    m.row[2] -= n.row[2];
    return m;
}

// Multiply one matrix into another.
constexpr matrix3x3 operator*=(matrix3x3 &m, const matrix3x3 &n) noexcept {
    m = m * n;
    return m;
}

// Check if two matrices are equal.
constexpr bool operator==(const matrix3x3 &m, const matrix3x3 &n) noexcept {
    return m.row[0] == n.row[0] &&
           m.row[1] == n.row[1] &&
           m.row[2] == n.row[2];
}

// Check if two matrices are different.
constexpr bool operator!=(const matrix3x3 &m, const matrix3x3 &n) noexcept {
    return m.row[0] != n.row[0] ||
           m.row[1] != n.row[1] ||
           m.row[2] != n.row[2];
}

// Check if a matrix is bigger than another component-wise.
constexpr bool operator>(const matrix3x3 &m, const matrix3x3 &n) noexcept {
    return m.row[0] > n.row[0] &&
           m.row[1] > n.row[1] &&
           m.row[2] > n.row[2];
}

// Check if a matrix is smaller than another component-wise.
constexpr bool operator<(const matrix3x3 &m, const matrix3x3 &n) noexcept {
    return m.row[0] < n.row[0] &&
           m.row[1] < n.row[1] &&
           m.row[2] < n.row[2];
}

// Check if a matrix is greater than or equal to another component-wise.
constexpr bool operator>=(const matrix3x3 &m, const matrix3x3 &n) noexcept {
    return m.row[0] >= n.row[0] &&
           m.row[1] >= n.row[1] &&
           m.row[2] >= n.row[2];
}

// Check if a matrix is less than or equal to another component-wise.
constexpr bool operator<=(const matrix3x3 &m, const matrix3x3 &n) noexcept {
    return m.row[0] <= n.row[0] &&
           m.row[1] <= n.row[1] &&
           m.row[2] <= n.row[2];
}

// Create a matrix with the given column vectors.
constexpr matrix3x3 matrix3x3_columns(const vector3 &v0,
                                      const vector3 &v1,
                                      const vector3 &v2) noexcept {
    return {
        vector3{v0.x, v1.x, v2.x},
        vector3{v0.y, v1.y, v2.y},
        vector3{v0.z, v1.z, v2.z}
    };
}

// Transpose of a 3x3 matrix.
constexpr matrix3x3 transpose(const matrix3x3 &m) noexcept {
    return {m.column(0), m.column(1), m.column(2)};
}

// Adjugate of a 3x3 matrix, i.e. the transpose of the cofactor matrix.
constexpr matrix3x3 adjugate_matrix(const matrix3x3 &m) noexcept {
    // Cofactors.
    auto c0 = cross(m[1], m[2]);
    auto c1 = cross(m[2], m[0]);
    auto c2 = cross(m[0], m[1]);
    // Transpose of cofactor matrix.
    return matrix3x3_columns(c0, c1, c2);
}

// Inverse of a 3x3 matrix or the zero matrix if `m` is non-invertible.
inline matrix3x3 inverse_matrix(const matrix3x3 &m) noexcept {
    auto det = m.determinant();
    scalar det_inv = 0;

    if (std::abs(det) > EDYN_EPSILON) {
        det_inv = scalar(1) / det;
    }

    return adjugate_matrix(m) * det_inv;
}

// Optimized inverse for symmetric 3x3 matrices.
constexpr matrix3x3 inverse_matrix_symmetric(const matrix3x3 &m) noexcept {
    EDYN_ASSERT(m[0][1] == m[1][0]);
    EDYN_ASSERT(m[0][2] == m[2][0]);
    EDYN_ASSERT(m[1][2] == m[2][1]);

    auto det = m.determinant();
    EDYN_ASSERT(det != 0);
    auto det_inv = scalar(1) / det;

    auto a11 = m[0][0], a12 = m[0][1], a13 = m[0][2];
    auto a22 = m[1][1], a23 = m[1][2];
    auto a33 = m[2][2];

    matrix3x3 m_inv{};

    m_inv[0][0] = det_inv * (a22 * a33 - a23 * a23);
    m_inv[0][1] = det_inv * (a13 * a23 - a12 * a33);
    m_inv[0][2] = det_inv * (a12 * a23 - a13 * a22);

    m_inv[1][0] = m_inv[0][1];
    m_inv[1][1] = det_inv * (a11 * a33 - a13 * a13);
    m_inv[1][2] = det_inv * (a12 * a13 - a11 * a23);

    m_inv[2][0] = m_inv[0][2];
    m_inv[2][1] = m_inv[1][2];
    m_inv[2][2] = det_inv * (a11 * a22 - a12 * a12);

    return m_inv;
}

// Matrix with given vector as diagonal.
constexpr matrix3x3 diagonal_matrix(const vector3 &v) noexcept {
    return {
        vector3{v.x, 0, 0},
        vector3{0, v.y, 0},
        vector3{0, 0, v.z}
    };
}

constexpr vector3 get_diagonal(const matrix3x3 &m) noexcept {
    return {m[0][0], m[1][1], m[2][2]};
}

// Equivalent to m * diagonal_matrix(v).
constexpr matrix3x3 scale_matrix(const matrix3x3 &m, const vector3 &v) noexcept {
    return {
        vector3{m.row[0].x * v.x, m.row[0].y * v.y, m.row[0].z * v.z},
        vector3{m.row[1].x * v.x, m.row[1].y * v.y, m.row[1].z * v.z},
        vector3{m.row[2].x * v.x, m.row[2].y * v.y, m.row[2].z * v.z}
    };
}

// Skew anti-symmetric matrix of a vector.
constexpr matrix3x3 skew_matrix(const vector3 &v) noexcept {
    return {
        vector3{0, -v.z, v.y},
        vector3{v.z, 0, -v.x},
        vector3{-v.y, v.x, 0}
    };
}

// Converts a quaternion into a rotation matrix.
constexpr matrix3x3 to_matrix3x3(const quaternion &q) noexcept {
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

// Converts a rotation matrix into a quaternion.
inline quaternion to_quaternion(const matrix3x3 &m) noexcept {
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

    quaternion quat;
    quat.w = t * (m[k][j] - m[j][k]);
    quat[i] = scalar(0.5) * s;
    quat[j] = t * (m[j][i] + m[i][j]);
    quat[k] = t * (m[k][i] + m[i][k]);

    return quat;
}

// Get XYZ Euler angles from a rotation matrix.
// Reference: Euler Angle Formulas - David Eberly, Geometric Tools
// https://www.geometrictools.com/Documentation/EulerAngles.pdf
inline vector3 get_euler_angles_xyz(const matrix3x3 &m) noexcept {
    if (m[0][2] < 1) {
        if (m[0][2] > -1) {
            return {
                std::atan2(-m[1][2], m[2][2]),
                std::asin(m[0][2]),
                std::atan2(-m[0][1], m[0][0])
            };
        } else {
            return {
                -std::atan2(m[1][0], m[1][1]),
                -pi_half,
                0
            };
        }
    } else {
        return {
            std::atan2(m[1][0], m[1][1]),
            pi_half,
            0
        };
    }
}

inline edyn::matrix3x3 rotation_matrix_x(scalar angle) {
    auto c = std::cos(angle);
    auto s = std::sin(angle);

    return {
        vector3{1, 0, 0},
        vector3{0, c, -s},
        vector3{0, s, c}
    };
}

inline edyn::matrix3x3 rotation_matrix_y(scalar angle) {
    auto c = std::cos(angle);
    auto s = std::sin(angle);

    return {
        vector3{c, 0, s},
        vector3{0, 1, 0},
        vector3{-s, 0, c}
    };
}

inline edyn::matrix3x3 rotation_matrix_z(scalar angle) {
    auto c = std::cos(angle);
    auto s = std::sin(angle);

    return {
        vector3{c, -s, 0},
        vector3{s, c, 0},
        vector3{0, 0, 1}
    };
}

inline edyn::matrix3x3 rotation_matrix_xyz(scalar angle_x, scalar angle_y, scalar angle_z) {
    auto cx = std::cos(angle_x);
    auto sx = std::sin(angle_x);
    auto cy = std::cos(angle_y);
    auto sy = std::sin(angle_y);
    auto cz = std::cos(angle_z);
    auto sz = std::sin(angle_z);

    return {
        vector3{cy*cz            , -cy*sz          , sy    },
        vector3{cz*sx*sy + cx*sz , cx*cz - sx*sy*sz, -cy*sx},
        vector3{-cx*cz*sy + sx*sz, cz*sx + cx*sy*sz, cx*cy }
    };
}

}

#endif // EDYN_MATH_MATRIX3X3_HPP
