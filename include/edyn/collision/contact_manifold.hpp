#ifndef EDYN_COLLISION_CONTACT_MANIFOLD_HPP
#define EDYN_COLLISION_CONTACT_MANIFOLD_HPP

#include <array>
#include "contact_point.hpp"

namespace edyn {

inline constexpr size_t max_contacts = 4;

struct contact_manifold {
    size_t num_points {0};
    std::array<contact_point, max_contacts> point;
};

}


#endif // EDYN_COLLISION_CONTACT_MANIFOLD_HPP