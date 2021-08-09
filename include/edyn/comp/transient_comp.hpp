#ifndef EDYN_COMP_TRANSIENT_COMP_HPP
#define EDYN_COMP_TRANSIENT_COMP_HPP

#include "edyn/comp/aabb.hpp"
#include "edyn/comp/linvel.hpp"
#include "edyn/comp/angvel.hpp"
#include "edyn/comp/inertia.hpp"
#include "edyn/comp/position.hpp"
#include "edyn/comp/orientation.hpp"
#include "edyn/constraints/constraint_impulse.hpp"
#include "edyn/collision/contact_point.hpp"

namespace edyn {

/**
 * Tuple of components that change often, usually in every step of the
 * simulation.
 */
static const auto transient_components = std::tuple<
    AABB,
    constraint_impulse,
    inertia_world_inv,
    angvel,
    linvel,
    position,
    orientation,
    contact_point
>{};

}

#endif // EDYN_COMP_TRANSIENT_COMP_HPP
