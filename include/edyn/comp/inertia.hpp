#ifndef EDYN_COMP_INERTIA_HPP
#define EDYN_COMP_INERTIA_HPP

#include "edyn/math/matrix3x3.hpp"

namespace edyn {

struct inertia : public vector3 {

};

struct inertia_inv : public vector3 {

};

struct inertia_world_inv : public matrix3x3 {

};

}

#endif // EDYN_COMP_INERTIA_HPP