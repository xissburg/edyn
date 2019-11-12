#ifndef EDYN_COMP_POSITION_HPP
#define EDYN_COMP_POSITION_HPP

#include "edyn/math/vector3.hpp"

namespace edyn {

struct position : public vector3 {

};

struct position_priv : public position {

};

}

#endif // EDYN_COMP_POSITION_HPP