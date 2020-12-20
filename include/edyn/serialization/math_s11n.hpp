#ifndef EDYN_SERIALIZATION_MATH_S11N_HPP
#define EDYN_SERIALIZATION_MATH_S11N_HPP

#include "edyn/math/vector3.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/matrix3x3.hpp"
#include "edyn/serialization/s11n_util.hpp"

namespace edyn {

template<typename Archive>
void serialize(Archive &archive, vector3 &v) {
    archive(v.x, v.y, v.z);
}

template<typename Archive>
void serialize(Archive &archive, quaternion &q) {
    archive(q.x, q.y, q.z, q.w);
}

template<typename Archive>
void serialize(Archive &archive, matrix3x3 &m) {
    archive(m.row);
}

}

#endif // EDYN_SERIALIZATION_MATH_S11N_HPP