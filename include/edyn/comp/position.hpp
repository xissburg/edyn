#ifndef EDYN_COMP_POSITION_HPP
#define EDYN_COMP_POSITION_HPP

#include "../math/vector3.hpp"

namespace edyn {

struct position {
    vector3 v;

    operator vector3&() {
        return v;
    }

    operator const vector3&() const {
        return v;
    }

    operator vector3() const {
        return v;
    }
};

}

#endif // EDYN_COMP_POSITION_HPP