#ifndef EDYN_COMP_INERTIA_HPP
#define EDYN_COMP_INERTIA_HPP

#include "edyn/math/matrix3x3.hpp"

namespace edyn {

struct inertia : public vector3 {
    inertia & operator=(const vector3 &v) {
        vector3::operator=(v);
        return *this;
    }
};

struct inertia_inv : public vector3 {
    inertia_inv & operator=(const vector3 &v) {
        vector3::operator=(v);
        return *this;
    }
};

struct inertia_world_inv : public matrix3x3 {
    inertia_world_inv & operator=(const matrix3x3 &m) {
        matrix3x3::operator=(m);
        return *this;
    }
};

}

#endif // EDYN_COMP_INERTIA_HPP