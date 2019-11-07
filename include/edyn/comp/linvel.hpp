#ifndef EDYN_COMP_VELOCITY_HPP
#define EDYN_COMP_VELOCITY_HPP

#include "../math/vector3.hpp"

namespace edyn {

struct linvel {
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

#endif // EDYN_COMP_VELOCITY_HPP