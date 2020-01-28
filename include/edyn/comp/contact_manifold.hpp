#ifndef EDYN_COLLISION_CONTACT_MANIFOLD_HPP
#define EDYN_COLLISION_CONTACT_MANIFOLD_HPP

#include <array>
#include "edyn/collision/contact_point.hpp"
#include "edyn/math/constants.hpp"

namespace edyn {

struct contact_manifold {
    size_t num_points {0};
    std::array<contact_point, max_contacts> point;
};

}


#endif // EDYN_COLLISION_CONTACT_MANIFOLD_HPP