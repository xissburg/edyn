#ifndef EDYN_CONSTRAINTS_CONSTRAINT_BODY_HPP
#define EDYN_CONSTRAINTS_CONSTRAINT_BODY_HPP

#include "edyn/math/matrix3x3.hpp"
#include "edyn/math/quaternion.hpp"
#include "edyn/math/vector3.hpp"

namespace edyn {

struct constraint_body {
    vector3 origin;
    vector3 pos;
    quaternion orn;
    vector3 linvel;
    vector3 angvel;
    scalar inv_m;
    matrix3x3 inv_I;
};

}

#endif // EDYN_CONSTRAINTS_CONSTRAINT_BODY_HPP
