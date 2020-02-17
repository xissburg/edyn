#ifndef EDYN_COMP_MATERIAL_HPP
#define EDYN_COMP_MATERIAL_HPP

#include "edyn/math/scalar.hpp"
#include "edyn/math/constants.hpp"

namespace edyn {

struct material {
    scalar restitution {0};
    scalar friction {0.5};
    scalar stiffness {large_scalar};
    scalar damping {large_scalar};

    bool use_contact_patch {false};
    bool is_tire {false};
    scalar speed_sensitivity {0.03};
    scalar tread_stiffness {2000000};
};

}

#endif // EDYN_COMP_MATERIAL_HPP