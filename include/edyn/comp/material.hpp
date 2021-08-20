#ifndef EDYN_COMP_MATERIAL_HPP
#define EDYN_COMP_MATERIAL_HPP

#include <limits>
#include "edyn/math/scalar.hpp"
#include "edyn/math/constants.hpp"

namespace edyn {

struct material_base {
    scalar restitution {0};
    scalar friction {0.5};
    scalar stiffness {large_scalar};
    scalar damping {large_scalar};
};

struct material : public material_base {
    using id_type = unsigned;
    id_type id {std::numeric_limits<id_type>::max()};
};

}

#endif // EDYN_COMP_MATERIAL_HPP